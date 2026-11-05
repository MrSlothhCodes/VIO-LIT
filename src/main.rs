use opencv::{
    calib3d,
    core,
    features2d::{self, ORB},
    highgui,
    imgproc,
    prelude::*,
    video,
    videoio,
};
use std::io::{BufRead, BufReader};

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion, Vector3};
use std::sync::mpsc;
use std::thread;

// ============================================================
// IMU plumbing (background only for this POC)
// ============================================================

#[derive(Debug, Clone, Copy)]
struct ImuSample {
    t_us: i64,
    ax: f32, ay: f32, az: f32,
    gx: f32, gy: f32, gz: f32,
}

fn read_serial_loop(port_name: &str, tx: mpsc::Sender<ImuSample>) {
    let port = match serialport::new(port_name, 921600)
        .timeout(std::time::Duration::from_millis(100))
        .open()
    {
        Ok(p) => p,
        Err(e) => {
            eprintln!("[imu] failed to open {}: {}", port_name, e);
            return;
        }
    };
    let mut reader = std::io::BufReader::new(port);
    let mut line = String::new();

    loop {
        line.clear();
        if reader.read_line(&mut line).is_err() {
            continue;
        }
        if line.starts_with("I,") {
            let parts: Vec<&str> = line.trim().split(',').collect();
            if parts.len() == 8 {
                if let (Ok(t_us), Ok(ax), Ok(ay), Ok(az), Ok(gx), Ok(gy), Ok(gz)) = (
                    parts[1].parse(),
                    parts[2].parse(),
                    parts[3].parse(),
                    parts[4].parse(),
                    parts[5].parse(),
                    parts[6].parse(),
                    parts[7].parse(),
                ) {
                    let sample = ImuSample { t_us, ax, ay, az, gx, gy, gz };
                    if tx.send(sample).is_err() {
                        return;
                    }
                }
            }
        }
    }
}

/// Background consumer: just drain the channel and print a throttled summary
/// so the serial thread never blocks. No MSCKF yet.
fn imu_drain_loop(rx: mpsc::Receiver<ImuSample>) {
    let mut count = 0u64;
    let mut last = std::time::Instant::now();
    let mut last_sample: Option<ImuSample> = None;
    for s in rx {
        count += 1;
        last_sample = Some(s);
        if last.elapsed().as_secs_f32() > 1.0 {
            if let Some(s) = last_sample {
                println!(
                    "[imu] {:.0} Hz  acc=({:+.2},{:+.2},{:+.2}) gyr=({:+.2},{:+.2},{:+.2})",
                    count as f32,
                    s.ax, s.ay, s.az, s.gx, s.gy, s.gz
                );
            }
            count = 0;
            last = std::time::Instant::now();
        }
    }
}

// ============================================================
// Helpers
// ============================================================

/// Convert OpenCV 3x3 Mat to nalgebra Matrix3
fn mat3_from_cv(m: &core::Mat) -> opencv::Result<Matrix3<f64>> {
    Ok(Matrix3::new(
        *m.at_2d::<f64>(0, 0)?, *m.at_2d::<f64>(0, 1)?, *m.at_2d::<f64>(0, 2)?,
        *m.at_2d::<f64>(1, 0)?, *m.at_2d::<f64>(1, 1)?, *m.at_2d::<f64>(1, 2)?,
        *m.at_2d::<f64>(2, 0)?, *m.at_2d::<f64>(2, 1)?, *m.at_2d::<f64>(2, 2)?,
    ))
}

/// Convert OpenCV 3x1 Mat to nalgebra Vector3
fn vec3_from_cv(m: &core::Mat) -> opencv::Result<Vector3<f64>> {
    Ok(Vector3::new(
        *m.at_2d::<f64>(0, 0)?,
        *m.at_2d::<f64>(1, 0)?,
        *m.at_2d::<f64>(2, 0)?,
    ))
}

// ============================================================
// Main: camera loop is the foreground; pose + path accumulate here.
// ============================================================

fn main() -> opencv::Result<()> {
    // ---- IMU in background ----
    let (tx, rx) = mpsc::channel::<ImuSample>();
    let port = std::env::var("IMU_PORT")
        .unwrap_or_else(|_| "/dev/cu.usbmodem1401".to_string());
    thread::Builder::new()
        .name("serial_imu".into())
        .spawn(move || read_serial_loop(&port, tx))
        .expect("spawn serial thread");
    thread::Builder::new()
        .name("imu_drain".into())
        .spawn(move || imu_drain_loop(rx))
        .expect("spawn imu drain thread");

    // ---- Camera ----
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Unable to open default camera!");
    }

    // ---- ORB ----
    let mut orb = ORB::create(
        500, 1.2, 8, 31, 0, 2,
        features2d::ORB_ScoreType::HARRIS_SCORE, 31, 20,
    )?;

    // ---- Intrinsics (replace with real calibration later) ----
    let fx = 500.0_f64;
    let fy = 500.0_f64;
    let cx = 320.0_f64;
    let cy = 240.0_f64;
    let camera_matrix = core::Mat::from_slice_2d(&[
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])?;

    // ---- Windows ----
    let feat_win = "Camera VIO Frontend";
    let path_win = "Path (top-down x-z)";
    highgui::named_window(feat_win, highgui::WINDOW_AUTOSIZE)?;
    highgui::named_window(path_win, highgui::WINDOW_AUTOSIZE)?;

    // ---- State ----
    let mut frame = core::Mat::default();
    let mut prev_gray = core::Mat::default();
    let mut gray = core::Mat::default();
    let mut keypoints = core::Vector::<core::KeyPoint>::new();
    let mut prev_pts = core::Vector::<core::Point2f>::new();

    // Pose accumulates as T_world_cam. Starts at identity.
    let mut pose_world_cam = Isometry3::<f64>::identity();
    // Path: list of camera centers in world frame (y=0 ignored for top-down view)
    let mut path: Vec<Vector3<f64>> = vec![Vector3::zeros()];
    // Only integrate motion when the camera actually moved (avoid jitter)
    let mut min_translation = 0.02; // arbitrary VO units

    // Top-down canvas
    let canvas_w = 800i32;
    let canvas_h = 800i32;
    let mut canvas = core::Mat::zeros(canvas_h, canvas_w, core::CV_8UC3)?.to_mat()?;
    let canvas_origin = (canvas_w as f32 / 2.0, canvas_h as f32 * 0.9);
    let canvas_scale = 100.0f32; // pixels per VO unit

    // Seed with first frame
    cam.read(&mut frame)?;
    if frame.empty() {
        return Ok(());
    }
    imgproc::cvt_color(
        &frame, &mut prev_gray, imgproc::COLOR_BGR2GRAY, 0,
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;

    loop {
        cam.read(&mut frame)?;
        if frame.empty() {
            break;
        }
        imgproc::cvt_color(
            &frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0,
            core::AlgorithmHint::ALGO_HINT_DEFAULT,
        )?;

        if prev_pts.is_empty() {
            // Re-detect
            keypoints.clear();
            let mut descriptors = core::Mat::default();
            orb.detect_and_compute(
                &gray, &core::Mat::default(),
                &mut keypoints, &mut descriptors, false,
            )?;
            for kp in keypoints.iter() {
                prev_pts.push(kp.pt());
            }
        } else {
            // Optical flow
            let mut status = core::Vector::<u8>::new();
            let mut err = core::Vector::<f32>::new();
            let mut next_pts = prev_pts.clone();
            video::calc_optical_flow_pyr_lk(
                &prev_gray, &gray,
                &prev_pts, &mut next_pts,
                &mut status, &mut err,
                core::Size::new(21, 21), 3,
                core::TermCriteria::new(
                    core::TermCriteria_Type::COUNT as i32
                        | core::TermCriteria_Type::EPS as i32,
                    30, 0.01,
                )?,
                video::OPTFLOW_USE_INITIAL_FLOW,
                1e-4,
            )?;

            let mut good_prev = core::Vector::<core::Point2f>::new();
            let mut good_next = core::Vector::<core::Point2f>::new();
            for i in 0..prev_pts.len() {
                if status.get(i)? != 0 {
                    good_prev.push(prev_pts.get(i)?);
                    good_next.push(next_pts.get(i)?);
                }
            }

            // Relative pose
            if good_prev.len() >= 8 {
                let mut mask = core::Mat::default();
                let mut r = core::Mat::default();
                let mut t = core::Mat::default();
                let e = calib3d::find_essential_mat(
                    &good_prev, &good_next, &camera_matrix,
                    calib3d::RANSAC, 0.999, 1.0, 1000, &mut mask,
                )?;
                let inliers = calib3d::recover_pose(
                    &e, &good_prev, &good_next,
                    &mut r, &mut t,
                    fx,
                    core::Point2d::new(cx, cy),
                    &mut mask,
                )?;

                if inliers as usize >= 8 {
                    // Sanity check: recompute optical-flow (recoverPose requires input/output semantics
                    // in some OpenCV versions; the returned good_next may have been filtered to inliers).
                    // Here we trust recover_pose's inliers count and the (r, t) matrices.
                    let r_mat = mat3_from_cv(&r)?;
                    let t_vec = vec3_from_cv(&t)?;

                    // t is unit-norm in monocular VO.
                    let t_norm = t_vec.norm();
                    if t_norm.is_finite() && t_norm > 1e-6 {
                        // Compose relative motion into absolute pose.
                        let rel = Isometry3::from_parts(
                            Translation3::from(t_vec),
                            UnitQuaternion::from_rotation_matrix(
                                &Rotation3::from_matrix_unchecked(r_mat),
                            ),
                        );
                        pose_world_cam = pose_world_cam * rel;

                        // Update path only when moved enough
                        let last = *path.last().unwrap();
                        if (pose_world_cam.translation.vector - last).norm() > min_translation {
                            path.push(pose_world_cam.translation.vector);
                        }
                    }
                }
            }

            // Draw tracked points on the frame
            for p in good_next.iter() {
                imgproc::circle(
                    &mut frame,
                    core::Point::new(p.x as i32, p.y as i32),
                    3,
                    core::Scalar::new(0.0, 255.0, 0.0, 0.0),
                    -1, 8, 0,
                )?;
            }

            prev_pts = good_next;
            if prev_pts.len() < 30 {
                prev_pts.clear();
            }
        }

        prev_gray = gray.clone();

        // ---- Draw path on canvas ----
        canvas = core::Mat::zeros(canvas_h, canvas_w, core::CV_8UC3)?.to_mat()?;
        // convert world (x, z) -> canvas (u, v). y is up in world; we use x-z plane.
        let project = |v: &Vector3<f64>| -> (i32, i32) {
            let u = canvas_origin.0 + (v.x as f32) * canvas_scale;
            let w = canvas_origin.1 - (v.z as f32) * canvas_scale;
            (u as i32, w as i32)
        };
        // Draw axes
        imgproc::circle(&mut canvas,
            core::Point::new(canvas_origin.0 as i32, canvas_origin.1 as i32),
            4, core::Scalar::new(255.0, 0.0, 0.0, 0.0), -1, 8, 0)?;

        if path.len() >= 2 {
            for i in 1..path.len() {
                let (u0, v0) = project(&path[i - 1]);
                let (u1, v1) = project(&path[i]);
                imgproc::line(
                    &mut canvas,
                    core::Point::new(u0, v0),
                    core::Point::new(u1, v1),
                    core::Scalar::new(0.0, 255.0, 0.0, 0.0),
                    2, 8, 0,
                )?;
            }
        }
        // Current pose marker
        let (uc, vc) = project(&pose_world_cam.translation.vector);
        imgproc::circle(
            &mut canvas,
            core::Point::new(uc, vc),
            6, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0,
        )?;

        highgui::imshow(feat_win, &frame)?;
        highgui::imshow(path_win, &canvas)?;
        let key = highgui::wait_key(1)?;
        if key == 27 || key == b'q' as i32 {
            break;
        }
    }

    Ok(())
}