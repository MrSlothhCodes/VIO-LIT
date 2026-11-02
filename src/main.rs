use opencv::{
    calib3d,
    core,
    features2d::{self, ORB},
    highgui,
    imgproc,
    prelude::*,
    video, // for calc_optical_flow_pyr_lk
    videoio,
};

fn main() -> opencv::Result<()> {
    // 1. Open the default camera
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    if !videoio::VideoCapture::is_opened(&cam)? {
        panic!("Unable to open default camera!");
    }

    // 2. Create the ORB detector (must be mutable for detect_and_compute)
    let mut orb = ORB::create(
        500, 1.2, 8, 31, 0, 2,
        features2d::ORB_ScoreType::HARRIS_SCORE, 31, 20,
    )?;

    // 3. Define Camera Intrinsic Matrix (K)
    // IMPORTANT: Replace these values with your camera's actual calibration!
    // These are placeholder values (fx, fy, cx, cy).
    let fx = 500.0;
    let fy = 500.0;
    let cx = 320.0;
    let cy = 240.0;
    let camera_matrix = core::Mat::from_slice_2d(&[
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ])?;

    let window = "Camera VIO Frontend";
    highgui::named_window(window, highgui::WINDOW_AUTOSIZE)?;

    // State variables
    let mut frame = core::Mat::default();
    let mut prev_gray = core::Mat::default();
    let mut gray = core::Mat::default();
    let mut keypoints = core::Vector::<core::KeyPoint>::new();
    let mut descriptors = core::Mat::default();

    // Points for optical flow
    let mut prev_pts = core::Vector::<core::Point2f>::new();
    let mut next_pts = core::Vector::<core::Point2f>::new();

    // Seed the tracker with the first frame
    cam.read(&mut frame)?;
    if frame.empty() {
        return Ok(());
    }
    imgproc::cvt_color(&frame, &mut prev_gray, imgproc::COLOR_BGR2GRAY, 0, core::AlgorithmHint::ALGO_HINT_DEFAULT)?;

    loop {
        cam.read(&mut frame)?;
        if frame.empty() {
            break;
        }

        imgproc::cvt_color(&frame, &mut gray, imgproc::COLOR_BGR2GRAY, 0, core::AlgorithmHint::ALGO_HINT_DEFAULT)?;

        // If we have no points to track (first iteration or all lost), detect with ORB
        if prev_pts.is_empty() {
            keypoints.clear();
            descriptors = core::Mat::default();
            orb.detect_and_compute(&gray, &core::Mat::default(), &mut keypoints, &mut descriptors, false)?;

            for kp in keypoints.iter() {
                prev_pts.push(kp.pt());
            }
        } else {
            // --- OPTICAL FLOW TRACKING ---
            let mut status = core::Vector::<u8>::new();
            let mut err = core::Vector::<f32>::new();
            next_pts = prev_pts.clone();

            video::calc_optical_flow_pyr_lk(
                &prev_gray,
                &gray,
                &prev_pts,
                &mut next_pts,
                &mut status,
                &mut err,
                core::Size::new(21, 21),
                3,
                core::TermCriteria::new(
                    core::TermCriteria_Type::COUNT as i32 | core::TermCriteria_Type::EPS as i32,
                    30,
                    0.01,
                )?,
                video::OPTFLOW_USE_INITIAL_FLOW,
                1e-4,
            )?;

            // Filter out points that were not tracked successfully
            let mut good_prev = core::Vector::<core::Point2f>::new();
            let mut good_next = core::Vector::<core::Point2f>::new();
            for i in 0..prev_pts.len() {
                if status.get(i)? != 0 {
                    good_prev.push(prev_pts.get(i)?);
                    good_next.push(next_pts.get(i)?);
                }
            }

            // --- RELATIVE POSE ESTIMATION ---
            if good_prev.len() >= 5 { // Minimum points for Essential Matrix
                //let mut e = core::Mat::default();
                let mut mask = core::Mat::default();
                let mut r = core::Mat::default();
                let mut t = core::Mat::default();

                // 1. Find Essential Matrix using RANSAC
                // Parameters: points1, points2, camera_matrix, method, prob, threshold, max_iters, mask
                let e = calib3d::find_essential_mat(
                    &good_prev,
                    &good_next,
                    &camera_matrix,
                    calib3d::RANSAC,
                    0.999,
                    1.0, // 1 pixel threshold
                    1000,
                    &mut mask,
                )?;

                // 2. Recover Pose (Rotation and Translation)
                // Parameters: E, points1, points2, camera_matrix, R, t, mask
                let inliers = calib3d::recover_pose(
                    &e,
                    &good_prev,
                    &good_next,
                 
                    &mut r,
                    &mut t,
                    fx,                             // f64 focal length
                    core::Point2d::new(cx, cy), 
                    &mut mask,
                )?;

                // Optional: Print inliers and translation direction to console
                // println!("Inliers: {}, t: {:?}", inliers, t);
            }

            // --- VISUALIZATION ---
            // Draw the tracked motion as lines
            for i in 0..good_prev.len() {
                let p0 = good_prev.get(i)?;
                let p1 = good_next.get(i)?;
                imgproc::line(
                    &mut frame,
                    core::Point::new(p0.x as i32, p0.y as i32),
                    core::Point::new(p1.x as i32, p1.y as i32),
                    core::Scalar::new(0.0, 255.0, 0.0, 0.0), // Green
                    1,
                    8,
                    0,
                )?;
            }

            // Prepare for next iteration: the current "next" points become "prev"
            prev_pts = good_next;

            // Re-detect if we lost too many features
            if prev_pts.len() < 30 {
                prev_pts.clear(); // This triggers ORB detection on the next loop
            }
        }

        // Update previous frame for optical flow
        prev_gray = gray.clone();

        highgui::imshow(window, &frame)?;
        if highgui::wait_key(1)? == 27 {
            break;
        }
    }

    Ok(())
}