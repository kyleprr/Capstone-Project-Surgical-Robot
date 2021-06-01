;; Auto-generated. Do not edit!


(when (boundp 'surgical_robot::Tracker)
  (if (not (find-package "SURGICAL_ROBOT"))
    (make-package "SURGICAL_ROBOT"))
  (shadow 'Tracker (find-package "SURGICAL_ROBOT")))
(unless (find-package "SURGICAL_ROBOT::TRACKER")
  (make-package "SURGICAL_ROBOT::TRACKER"))

(in-package "ROS")
;;//! \htmlinclude Tracker.msg.html


(defclass surgical_robot::Tracker
  :super ros::object
  :slots (_x _y _z _error_x _error_y _error_z _flag1 _flag2 _flag3 ))

(defmethod surgical_robot::Tracker
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:error_x __error_x) 0.0)
    ((:error_y __error_y) 0.0)
    ((:error_z __error_z) 0.0)
    ((:flag1 __flag1) nil)
    ((:flag2 __flag2) nil)
    ((:flag3 __flag3) nil)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _error_x (float __error_x))
   (setq _error_y (float __error_y))
   (setq _error_z (float __error_z))
   (setq _flag1 __flag1)
   (setq _flag2 __flag2)
   (setq _flag3 __flag3)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:error_x
   (&optional __error_x)
   (if __error_x (setq _error_x __error_x)) _error_x)
  (:error_y
   (&optional __error_y)
   (if __error_y (setq _error_y __error_y)) _error_y)
  (:error_z
   (&optional __error_z)
   (if __error_z (setq _error_z __error_z)) _error_z)
  (:flag1
   (&optional __flag1)
   (if __flag1 (setq _flag1 __flag1)) _flag1)
  (:flag2
   (&optional __flag2)
   (if __flag2 (setq _flag2 __flag2)) _flag2)
  (:flag3
   (&optional __flag3)
   (if __flag3 (setq _flag3 __flag3)) _flag3)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _error_x
    8
    ;; float64 _error_y
    8
    ;; float64 _error_z
    8
    ;; bool _flag1
    1
    ;; bool _flag2
    1
    ;; bool _flag3
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error_x
       (sys::poke _error_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error_y
       (sys::poke _error_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _error_z
       (sys::poke _error_z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _flag1
       (if _flag1 (write-byte -1 s) (write-byte 0 s))
     ;; bool _flag2
       (if _flag2 (write-byte -1 s) (write-byte 0 s))
     ;; bool _flag3
       (if _flag3 (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error_x
     (setq _error_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error_y
     (setq _error_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _error_z
     (setq _error_z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _flag1
     (setq _flag1 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flag2
     (setq _flag2 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flag3
     (setq _flag3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get surgical_robot::Tracker :md5sum-) "b283d7d9a91916dac4010d28ee78ee60")
(setf (get surgical_robot::Tracker :datatype-) "surgical_robot/Tracker")
(setf (get surgical_robot::Tracker :definition-)
      "# message type to describe the tracking information of the blocks
# to be published as a topic

float64 x  # x coordinate in the world
float64 y  # y coordinate in the world
float64 z  # z coordinate in the world
float64 error_x
float64 error_y
float64 error_z
bool flag1
bool flag2
bool flag3

")



(provide :surgical_robot/Tracker "b283d7d9a91916dac4010d28ee78ee60")


