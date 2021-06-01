; Auto-generated. Do not edit!


(cl:in-package surgical_robot-msg)


;//! \htmlinclude Tracker.msg.html

(cl:defclass <Tracker> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (error_x
    :reader error_x
    :initarg :error_x
    :type cl:float
    :initform 0.0)
   (error_y
    :reader error_y
    :initarg :error_y
    :type cl:float
    :initform 0.0)
   (error_z
    :reader error_z
    :initarg :error_z
    :type cl:float
    :initform 0.0)
   (flag1
    :reader flag1
    :initarg :flag1
    :type cl:boolean
    :initform cl:nil)
   (flag2
    :reader flag2
    :initarg :flag2
    :type cl:boolean
    :initform cl:nil)
   (flag3
    :reader flag3
    :initarg :flag3
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Tracker (<Tracker>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tracker>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tracker)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surgical_robot-msg:<Tracker> is deprecated: use surgical_robot-msg:Tracker instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:x-val is deprecated.  Use surgical_robot-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:y-val is deprecated.  Use surgical_robot-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:z-val is deprecated.  Use surgical_robot-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'error_x-val :lambda-list '(m))
(cl:defmethod error_x-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:error_x-val is deprecated.  Use surgical_robot-msg:error_x instead.")
  (error_x m))

(cl:ensure-generic-function 'error_y-val :lambda-list '(m))
(cl:defmethod error_y-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:error_y-val is deprecated.  Use surgical_robot-msg:error_y instead.")
  (error_y m))

(cl:ensure-generic-function 'error_z-val :lambda-list '(m))
(cl:defmethod error_z-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:error_z-val is deprecated.  Use surgical_robot-msg:error_z instead.")
  (error_z m))

(cl:ensure-generic-function 'flag1-val :lambda-list '(m))
(cl:defmethod flag1-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:flag1-val is deprecated.  Use surgical_robot-msg:flag1 instead.")
  (flag1 m))

(cl:ensure-generic-function 'flag2-val :lambda-list '(m))
(cl:defmethod flag2-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:flag2-val is deprecated.  Use surgical_robot-msg:flag2 instead.")
  (flag2 m))

(cl:ensure-generic-function 'flag3-val :lambda-list '(m))
(cl:defmethod flag3-val ((m <Tracker>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surgical_robot-msg:flag3-val is deprecated.  Use surgical_robot-msg:flag3 instead.")
  (flag3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tracker>) ostream)
  "Serializes a message object of type '<Tracker>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'error_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag1) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag2) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag3) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tracker>) istream)
  "Deserializes a message object of type '<Tracker>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flag1) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flag2) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flag3) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tracker>)))
  "Returns string type for a message object of type '<Tracker>"
  "surgical_robot/Tracker")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tracker)))
  "Returns string type for a message object of type 'Tracker"
  "surgical_robot/Tracker")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tracker>)))
  "Returns md5sum for a message object of type '<Tracker>"
  "b283d7d9a91916dac4010d28ee78ee60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tracker)))
  "Returns md5sum for a message object of type 'Tracker"
  "b283d7d9a91916dac4010d28ee78ee60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tracker>)))
  "Returns full string definition for message of type '<Tracker>"
  (cl:format cl:nil "# message type to describe the tracking information of the blocks~%# to be published as a topic~%~%float64 x  # x coordinate in the world~%float64 y  # y coordinate in the world~%float64 z  # z coordinate in the world~%float64 error_x~%float64 error_y~%float64 error_z~%bool flag1~%bool flag2~%bool flag3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tracker)))
  "Returns full string definition for message of type 'Tracker"
  (cl:format cl:nil "# message type to describe the tracking information of the blocks~%# to be published as a topic~%~%float64 x  # x coordinate in the world~%float64 y  # y coordinate in the world~%float64 z  # z coordinate in the world~%float64 error_x~%float64 error_y~%float64 error_z~%bool flag1~%bool flag2~%bool flag3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tracker>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tracker>))
  "Converts a ROS message object to a list"
  (cl:list 'Tracker
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':error_x (error_x msg))
    (cl:cons ':error_y (error_y msg))
    (cl:cons ':error_z (error_z msg))
    (cl:cons ':flag1 (flag1 msg))
    (cl:cons ':flag2 (flag2 msg))
    (cl:cons ':flag3 (flag3 msg))
))
