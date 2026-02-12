#!/usr/bin/env roseus
;;;
;;; ROS 2 version of test-add-two-ints-server-groupname.l
;;; Excluded: connection-header assertion (not available in ROS 2 DDS)

(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

(require :unittest "lib/llib/unittest.l")
(ros::load-ros-manifest "roseus")
;;;
;;;
(init-unit-test)

(defun add-two-ints (req)
  (let ((m (send req :response)))
    (warning-message 2 "Returning [~d + ~d = ~d]~%"
                     (send req :a) (send req :b)
                     (+ (send req :a) (send req :b)))
    (send m :sum  (+ (send req :a) (send req :b)))
    m))

(deftest test-add-two-ints-server
  (ros::roseus "add_two_ints_server")
  (ros::advertise-service "add_two_ints" roseus::AddTwoInts #'add-two-ints
                          :groupname "add_two_ints_server_groupname")

  ;; Spin long enough for DDS discovery + client calls (ROS 2 discovery is slower than ROS 1)
  (ros::rate 10)
  (dotimes (i 150)
    (ros::spin-once "add_two_ints_server_groupname")
    (ros::sleep))
  )

(ros::roseus "add_two_ints_server_init")
(ros::create-nodehandle "add_two_ints_server_groupname")
(run-all-tests)

(exit)
