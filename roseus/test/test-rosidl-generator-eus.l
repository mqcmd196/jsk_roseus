#!/usr/bin/env eus
;;
;; Black-box test for rosidl_generator_eus
;; Tests generated .l files from roseus's own msg/srv definitions:
;;   instantiation, accessors, serialize/deserialize round-trip
;;

;; ============================================================
;; Minimal roseus environment (no C extension needed)
;; ============================================================
(unless (find-package "ROS") (make-package "ROS"))
(in-package "ROS")

(shadow 'object *package*)
(defclass ros::object
  :super propertied-object
  :slots ())
(defmethod ros::object
  (:init () self)
  (:md5sum- () (get (class self) :md5sum-))
  (:datatype- () (get (class self) :datatype-)))

;; Stub: ros::roseus-add-msgs / ros::roseus-add-srvs / ros::roseus-add-actions (no-op for test)
(defun ros::roseus-add-msgs (pkg)
  (warning-message 3 ";; [test stub] ros::roseus-add-msgs ~A (skipped)~%" pkg))
(defun ros::roseus-add-srvs (pkg)
  (warning-message 3 ";; [test stub] ros::roseus-add-srvs ~A (skipped)~%" pkg))
(defun ros::roseus-add-actions (pkg)
  (warning-message 3 ";; [test stub] ros::roseus-add-actions ~A (skipped)~%" pkg))

(in-package "USER")

;; ============================================================
;; Stubs for cross-package message types
;; ============================================================

;; -- builtin_interfaces/Time --
(unless (find-package "BUILTIN_INTERFACES")
  (make-package "BUILTIN_INTERFACES"))
(unless (find-package "BUILTIN_INTERFACES::TIME")
  (make-package "BUILTIN_INTERFACES::TIME"))

(defclass builtin_interfaces::Time
  :super ros::object
  :slots (_sec _nanosec))
(defmethod builtin_interfaces::Time
  (:init (&key ((:sec __sec) 0) ((:nanosec __nanosec) 0))
   (send-super :init)
   (setq _sec __sec _nanosec __nanosec) self)
  (:sec (&optional v) (if v (setq _sec v)) _sec)
  (:nanosec (&optional v) (if v (setq _nanosec v)) _nanosec)
  (:serialization-length () 8)
  (:serialize (&optional strm)
   (let ((s (if strm strm (make-string-output-stream 8))))
     (write-long _sec s) (write-long _nanosec s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize (buf &optional (ptr- 0))
   (setq _sec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   (setq _nanosec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   self))

;; -- builtin_interfaces/Duration --
(unless (find-package "BUILTIN_INTERFACES::DURATION")
  (make-package "BUILTIN_INTERFACES::DURATION"))

(defclass builtin_interfaces::Duration
  :super ros::object
  :slots (_sec _nanosec))
(defmethod builtin_interfaces::Duration
  (:init (&key ((:sec __sec) 0) ((:nanosec __nanosec) 0))
   (send-super :init)
   (setq _sec __sec _nanosec __nanosec) self)
  (:sec (&optional v) (if v (setq _sec v)) _sec)
  (:nanosec (&optional v) (if v (setq _nanosec v)) _nanosec)
  (:serialization-length () 8)
  (:serialize (&optional strm)
   (let ((s (if strm strm (make-string-output-stream 8))))
     (write-long _sec s) (write-long _nanosec s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize (buf &optional (ptr- 0))
   (setq _sec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   (setq _nanosec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   self))

;; -- std_msgs/Header --
(unless (find-package "STD_MSGS") (make-package "STD_MSGS"))
(unless (find-package "STD_MSGS::HEADER") (make-package "STD_MSGS::HEADER"))

(defclass std_msgs::Header
  :super ros::object
  :slots (_stamp _frame_id))
(defmethod std_msgs::Header
  (:init (&key ((:stamp __stamp) (instance builtin_interfaces::Time :init))
               ((:frame_id __frame_id) ""))
   (send-super :init) (setq _stamp __stamp _frame_id __frame_id) self)
  (:stamp (&rest args)
   (if (keywordp (car args)) (send* _stamp args)
     (progn (if args (setq _stamp (car args))) _stamp)))
  (:frame_id (&optional v) (if v (setq _frame_id v)) _frame_id)
  (:serialization-length () (+ 8 4 (length _frame_id)))
  (:serialize (&optional strm)
   (let ((s (if strm strm (make-string-output-stream (send self :serialization-length)))))
     (send _stamp :serialize s)
     (write-long (length _frame_id) s) (princ _frame_id s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize (buf &optional (ptr- 0))
   (send _stamp :deserialize buf ptr-) (incf ptr- 8)
   (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   self))

;; -- std_msgs/String --
;; 'String' conflicts with EusLisp built-in string class; must shadow first
(shadow 'String (find-package "STD_MSGS"))
(unless (find-package "STD_MSGS::STRING") (make-package "STD_MSGS::STRING"))

(defclass std_msgs::String
  :super ros::object
  :slots (_data))
(defmethod std_msgs::String
  (:init (&key ((:data __data) ""))
   (send-super :init) (setq _data (string __data)) self)
  (:data (&optional v) (if v (setq _data v)) _data)
  (:serialization-length () (+ 4 (length _data)))
  (:serialize (&optional strm)
   (let ((s (if strm strm (make-string-output-stream (send self :serialization-length)))))
     (write-long (length _data) s) (princ _data s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize (buf &optional (ptr- 0))
   (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   self))

;; -- unique_identifier_msgs/UUID --
(unless (find-package "UNIQUE_IDENTIFIER_MSGS")
  (make-package "UNIQUE_IDENTIFIER_MSGS"))
(unless (find-package "UNIQUE_IDENTIFIER_MSGS::UUID")
  (make-package "UNIQUE_IDENTIFIER_MSGS::UUID"))

(defclass unique_identifier_msgs::UUID
  :super ros::object
  :slots (_uuid))
(defmethod unique_identifier_msgs::UUID
  (:init (&key ((:uuid __uuid) (make-array 16 :initial-element 0 :element-type :char)))
   (send-super :init) (setq _uuid __uuid) self)
  (:uuid (&optional v) (if v (setq _uuid v)) _uuid)
  (:serialization-length () 16)
  (:serialize (&optional strm)
   (let ((s (if strm strm (make-string-output-stream 16))))
     (princ _uuid s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize (buf &optional (ptr- 0))
   (setq _uuid (make-array 16 :element-type :char))
   (replace _uuid buf :start2 ptr-) (incf ptr- 16)
   self)
  (:serialization-length-cdr () (+ 4 16))
  (:serialize-cdr (&optional strm)
   (let ((s (if strm strm (make-string-output-stream 20))))
     (write-byte 0 s) (write-byte 1 s) (write-byte 0 s) (write-byte 0 s)
     (princ _uuid s)
     (if (null strm) (get-output-stream-string s))))
  (:deserialize-cdr (buf &optional (ptr- 0))
   (incf ptr- 4)
   (setq _uuid (make-array 16 :element-type :char))
   (replace _uuid buf :start2 ptr-) (incf ptr- 16)
   self))

;; ============================================================
;; Test infrastructure
;; ============================================================
(require :unittest "lib/llib/unittest.l")
(init-unit-test)

;; ============================================================
;; Load generated .l files from roseus package
;; ============================================================
;; Pre-create ROSEUS package and shadow built-in conflicting names
;; (roseus::String conflicts with EusLisp built-in string class)
(unless (find-package "ROSEUS") (make-package "ROSEUS"))
(shadow 'String (find-package "ROSEUS"))

;; lisp::*eustop-argument* = (eus <script.l> <dir-arg>)
(if (< (length lisp::*eustop-argument*) 3)
  (progn
    (format *error-output* "Usage: eus test-rosidl-generator-eus.l <path-to-generated-l-dir>~%")
    (exit 1))
  (setq *test-l-dir* (elt lisp::*eustop-argument* 2)))
(format t ";; Loading generated .l files from: ~A~%" *test-l-dir*)

(dolist (subdir '("msg" "srv" "action"))
  (let ((dir (format nil "~A/~A/" *test-l-dir* subdir)))
    (when (probe-file dir)
      (dolist (f (directory dir))
        (when (string= (pathname-type f) "l")
          (let ((path (format nil "~A~A" dir f)))
            (format t ";;   loading ~A/~A~%" subdir f)
            (load path)))))))

;; ============================================================
;; Helper: serialize/deserialize round-trip
;; ============================================================
(defun round-trip (msg)
  "Serialize msg, create new instance of same class, deserialize, return it."
  (let* ((buf (send msg :serialize))
         (msg2 (instance (class msg) :init)))
    (send msg2 :deserialize buf)
    msg2))

;; ============================================================
;; Test: roseus/String
;; ============================================================
(deftest test-string-init
  (let ((msg (instance roseus::String :init)))
    (assert (stringp (send msg :data)) "default data is string")
    (assert (string= "" (send msg :data)) "default data is empty string")))

(deftest test-string-accessor
  (let ((msg (instance roseus::String :init :data "hello")))
    (assert (string= "hello" (send msg :data)) "init keyword sets data")
    (send msg :data "world")
    (assert (string= "world" (send msg :data)) "accessor sets data")))

(deftest test-string-roundtrip
  (let* ((msg (instance roseus::String :init :data "test123"))
         (msg2 (round-trip msg)))
    (assert (string= "test123" (send msg2 :data))
            "round-trip preserves string data")))

(deftest test-string-empty-roundtrip
  (let* ((msg (instance roseus::String :init))
         (msg2 (round-trip msg)))
    (assert (string= "" (send msg2 :data))
            "round-trip preserves empty string")))

;; ============================================================
;; Test: roseus/StringStamped2 (nested: Header + string)
;; ============================================================
(deftest test-stringstamped-init
  (let ((msg (instance roseus::StringStamped2 :init)))
    (assert (derivedp (send msg :header) std_msgs::Header) "header is Header type")
    (assert (stringp (send msg :data)) "data is string")))

(deftest test-stringstamped-roundtrip
  (let* ((hdr (instance std_msgs::Header :init :frame_id "map"))
         (msg (instance roseus::StringStamped2 :init :header hdr :data "hello_stamped"))
         (msg2 (round-trip msg)))
    (assert (string= "map" (send (send msg2 :header) :frame_id))
            "rt header frame_id")
    (assert (string= "hello_stamped" (send msg2 :data))
            "rt data")))

;; ============================================================
;; Test: roseus/TestName2 (self-referencing: roseus/StringStamped2)
;; ============================================================
(deftest test-testname-init
  (let ((msg (instance roseus::TestName2 :init)))
    (assert (derivedp (send msg :name) roseus::StringStamped2)
            "name field is StringStamped2 type")))

(deftest test-testname-roundtrip
  (let* ((inner (instance roseus::StringStamped2 :init
                          :header (instance std_msgs::Header :init :frame_id "odom")
                          :data "inner_data"))
         (msg (instance roseus::TestName2 :init :name inner))
         (msg2 (round-trip msg)))
    (assert (string= "odom"
                      (send (send (send msg2 :name) :header) :frame_id))
            "rt nested header frame_id")
    (assert (string= "inner_data" (send (send msg2 :name) :data))
            "rt nested data")))

;; ============================================================
;; Test: roseus/TestFixedArray (fixed-length arrays)
;; ============================================================
(deftest test-fixedarray-init
  (let ((msg (instance roseus::TestFixedArray :init)))
    (assert (= 3 (length (send msg :float32_data))) "float32_data fixed len 3")
    (assert (= 3 (length (send msg :float64_data))) "float64_data fixed len 3")
    (assert (= 3 (length (send msg :int32_data))) "int32_data fixed len 3")
    (assert (= 17 (length (send msg :uint8_data))) "uint8_data fixed len 17")
    (assert (= 2 (length (send msg :bool_data))) "bool_data fixed len 2")
    (assert (= 2 (length (send msg :time_data))) "time_data fixed len 2")
    (assert (= 2 (length (send msg :duration_data))) "duration_data fixed len 2")
    (assert (= 2 (length (send msg :string_data))) "string_data fixed len 2")))

(deftest test-fixedarray-roundtrip
  (let* ((msg (instance roseus::TestFixedArray :init
                        :float32_data (float-vector 1.0 2.0 3.0)
                        :int32_data (integer-vector 10 20 30)
                        :int64_data (integer-vector 100 200 300)))
         (msg2 (round-trip msg)))
    ;; float32
    (dotimes (i 3)
      (assert (< (abs (- (elt (float-vector 1.0 2.0 3.0) i)
                         (elt (send msg2 :float32_data) i))) 0.001)
              (format nil "rt float32_data[~A]" i)))
    ;; int32
    (dotimes (i 3)
      (assert (= (elt (integer-vector 10 20 30) i)
                 (elt (send msg2 :int32_data) i))
              (format nil "rt int32_data[~A]" i)))
    ;; int64
    (dotimes (i 3)
      (assert (= (elt (integer-vector 100 200 300) i)
                 (elt (send msg2 :int64_data) i))
              (format nil "rt int64_data[~A]" i)))))

;; ============================================================
;; Test: roseus/TestVariableArray (dynamic-length arrays)
;; ============================================================
(deftest test-variablearray-roundtrip
  (let* ((msg (instance roseus::TestVariableArray :init
                        :float32_data (float-vector 0.5 1.5)
                        :int32_data (integer-vector 7 8 9)
                        :uint8_data (let ((a (make-array 3 :element-type :char)))
                                      (dotimes (i 3) (setf (elt a i) (+ i 10))) a)))
         (msg2 (round-trip msg)))
    ;; float32
    (assert (= 2 (length (send msg2 :float32_data))) "rt var float32 len")
    (dotimes (i 2)
      (assert (< (abs (- (elt (float-vector 0.5 1.5) i)
                         (elt (send msg2 :float32_data) i))) 0.001)
              (format nil "rt var float32[~A]" i)))
    ;; int32
    (assert (= 3 (length (send msg2 :int32_data))) "rt var int32 len")
    (dotimes (i 3)
      (assert (= (elt (integer-vector 7 8 9) i)
                 (elt (send msg2 :int32_data) i))
              (format nil "rt var int32[~A]" i)))
    ;; uint8
    (assert (= 3 (length (send msg2 :uint8_data))) "rt var uint8 len")))

;; ============================================================
;; Test: roseus/AddTwoInts (service)
;; ============================================================
(deftest test-add-two-ints-service
  (assert (boundp 'roseus::AddTwoInts) "service class exists")
  (assert (boundp 'roseus::AddTwoIntsRequest) "request class exists")
  (assert (boundp 'roseus::AddTwoIntsResponse) "response class exists")
  ;; request/response linkage
  (assert (eq roseus::AddTwoIntsRequest
              (get roseus::AddTwoInts :request))
          "service :request link")
  (assert (eq roseus::AddTwoIntsResponse
              (get roseus::AddTwoInts :response))
          "service :response link")
  ;; :response method on request
  (let ((req (instance roseus::AddTwoIntsRequest :init)))
    (assert (derivedp (send req :response) roseus::AddTwoIntsResponse)
            "request :response creates response")))

(deftest test-add-two-ints-roundtrip
  (let* ((req (instance roseus::AddTwoIntsRequest :init :a 100 :b 200))
         (req2 (round-trip req)))
    (assert (= 100 (send req2 :a)) "rt request a")
    (assert (= 200 (send req2 :b)) "rt request b"))
  (let* ((res (instance roseus::AddTwoIntsResponse :init :sum 300))
         (res2 (round-trip res)))
    (assert (= 300 (send res2 :sum)) "rt response sum")))

;; ============================================================
;; Test: roseus/StringString (service)
;; ============================================================
(deftest test-string-string-roundtrip
  (let* ((req (instance roseus::StringStringRequest :init :str "hello"))
         (req2 (round-trip req)))
    (assert (string= "hello" (send req2 :str)) "rt string srv request"))
  (let* ((res (instance roseus::StringStringResponse :init :str "world"))
         (res2 (round-trip res)))
    (assert (string= "world" (send res2 :str)) "rt string srv response")))

;; ============================================================
;; Test: datatype- property
;; ============================================================
(deftest test-datatype-property
  (let ((msg (instance roseus::String :init)))
    (assert (string= "roseus/msg/String" (send msg :datatype-))
            "String datatype-"))
  (assert (string= "roseus/srv/AddTwoInts"
                    (get roseus::AddTwoInts :datatype-))
          "AddTwoInts service datatype-"))

;; ============================================================
;; Test: roseus/Fibonacci (action)
;; ============================================================
(deftest test-fibonacci-action-classes-exist
  (assert (boundp 'roseus::Fibonacci) "action class exists")
  (assert (boundp 'roseus::Fibonacci_Goal) "goal class exists")
  (assert (boundp 'roseus::Fibonacci_Result) "result class exists")
  (assert (boundp 'roseus::Fibonacci_Feedback) "feedback class exists")
  (assert (boundp 'roseus::Fibonacci_SendGoal) "send-goal service class exists")
  (assert (boundp 'roseus::Fibonacci_SendGoalRequest) "send-goal request class exists")
  (assert (boundp 'roseus::Fibonacci_SendGoalResponse) "send-goal response class exists")
  (assert (boundp 'roseus::Fibonacci_GetResult) "get-result service class exists")
  (assert (boundp 'roseus::Fibonacci_GetResultRequest) "get-result request class exists")
  (assert (boundp 'roseus::Fibonacci_GetResultResponse) "get-result response class exists")
  (assert (boundp 'roseus::Fibonacci_FeedbackMessage) "feedback message class exists"))

(deftest test-fibonacci-action-properties
  ;; Action meta-class properties
  (assert (eq roseus::Fibonacci_Goal
              (get roseus::Fibonacci :goal))
          "action :goal link")
  (assert (eq roseus::Fibonacci_Result
              (get roseus::Fibonacci :result))
          "action :result link")
  (assert (eq roseus::Fibonacci_Feedback
              (get roseus::Fibonacci :feedback))
          "action :feedback link")
  (assert (eq roseus::Fibonacci_SendGoal
              (get roseus::Fibonacci :send-goal-service))
          "action :send-goal-service link")
  (assert (eq roseus::Fibonacci_GetResult
              (get roseus::Fibonacci :get-result-service))
          "action :get-result-service link")
  (assert (eq roseus::Fibonacci_FeedbackMessage
              (get roseus::Fibonacci :feedback-message))
          "action :feedback-message link")
  ;; Datatype
  (assert (string= "roseus/action/Fibonacci"
                    (get roseus::Fibonacci :datatype-))
          "Fibonacci action datatype-"))

(deftest test-fibonacci-goal-roundtrip
  (let* ((msg (instance roseus::Fibonacci_Goal :init :order 5))
         (msg2 (round-trip msg)))
    (assert (= 5 (send msg2 :order)) "rt goal order")))

(deftest test-fibonacci-result-roundtrip
  (let* ((msg (instance roseus::Fibonacci_Result :init
                        :sequence (integer-vector 0 1 1 2 3 5)))
         (msg2 (round-trip msg)))
    (assert (= 6 (length (send msg2 :sequence))) "rt result sequence length")
    (dotimes (i 6)
      (assert (= (elt (integer-vector 0 1 1 2 3 5) i)
                 (elt (send msg2 :sequence) i))
              (format nil "rt result sequence[~A]" i)))))

(deftest test-fibonacci-feedback-roundtrip
  (let* ((msg (instance roseus::Fibonacci_Feedback :init
                        :sequence (integer-vector 0 1 1)))
         (msg2 (round-trip msg)))
    (assert (= 3 (length (send msg2 :sequence))) "rt feedback sequence length")
    (dotimes (i 3)
      (assert (= (elt (integer-vector 0 1 1) i)
                 (elt (send msg2 :sequence) i))
              (format nil "rt feedback sequence[~A]" i)))))

(deftest test-fibonacci-sendgoal-request-roundtrip
  (let* ((uuid (make-array 16 :element-type :char))
         (goal (instance roseus::Fibonacci_Goal :init :order 10))
         (req (instance roseus::Fibonacci_SendGoalRequest :init
                        :goal_id (instance unique_identifier_msgs::UUID :init :uuid uuid)
                        :goal goal))
         (req2 (round-trip req)))
    (assert (= 10 (send (send req2 :goal) :order)) "rt sendgoal request goal order")
    (assert (= 16 (length (send (send req2 :goal_id) :uuid))) "rt sendgoal request uuid length")))

(deftest test-fibonacci-getresult-response-roundtrip
  (let* ((result (instance roseus::Fibonacci_Result :init
                           :sequence (integer-vector 0 1 1 2 3)))
         (resp (instance roseus::Fibonacci_GetResultResponse :init
                         :status 4  ;; SUCCEEDED
                         :result result))
         (resp2 (round-trip resp)))
    (assert (= 4 (send resp2 :status)) "rt getresult response status")
    (assert (= 5 (length (send (send resp2 :result) :sequence)))
            "rt getresult response result sequence length")))

(deftest test-fibonacci-sendgoal-service
  ;; SendGoal service linkage
  (assert (eq roseus::Fibonacci_SendGoalRequest
              (get roseus::Fibonacci_SendGoal :request))
          "sendgoal service :request link")
  (assert (eq roseus::Fibonacci_SendGoalResponse
              (get roseus::Fibonacci_SendGoal :response))
          "sendgoal service :response link")
  ;; :response method on request
  (let ((req (instance roseus::Fibonacci_SendGoalRequest :init)))
    (assert (derivedp (send req :response) roseus::Fibonacci_SendGoalResponse)
            "sendgoal request :response creates response")))

(deftest test-fibonacci-getresult-service
  ;; GetResult service linkage
  (assert (eq roseus::Fibonacci_GetResultRequest
              (get roseus::Fibonacci_GetResult :request))
          "getresult service :request link")
  (assert (eq roseus::Fibonacci_GetResultResponse
              (get roseus::Fibonacci_GetResult :response))
          "getresult service :response link"))

;; ============================================================
;; Run all tests
;; ============================================================
(run-all-tests)
(exit 0)
