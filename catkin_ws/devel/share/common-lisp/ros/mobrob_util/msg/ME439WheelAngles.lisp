; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439WheelAngles.msg.html

(cl:defclass <ME439WheelAngles> (roslisp-msg-protocol:ros-message)
  ((ang_left
    :reader ang_left
    :initarg :ang_left
    :type cl:float
    :initform 0.0)
   (ang_right
    :reader ang_right
    :initarg :ang_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass ME439WheelAngles (<ME439WheelAngles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439WheelAngles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439WheelAngles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439WheelAngles> is deprecated: use mobrob_util-msg:ME439WheelAngles instead.")))

(cl:ensure-generic-function 'ang_left-val :lambda-list '(m))
(cl:defmethod ang_left-val ((m <ME439WheelAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:ang_left-val is deprecated.  Use mobrob_util-msg:ang_left instead.")
  (ang_left m))

(cl:ensure-generic-function 'ang_right-val :lambda-list '(m))
(cl:defmethod ang_right-val ((m <ME439WheelAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:ang_right-val is deprecated.  Use mobrob_util-msg:ang_right instead.")
  (ang_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439WheelAngles>) ostream)
  "Serializes a message object of type '<ME439WheelAngles>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ang_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ang_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439WheelAngles>) istream)
  "Deserializes a message object of type '<ME439WheelAngles>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ang_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439WheelAngles>)))
  "Returns string type for a message object of type '<ME439WheelAngles>"
  "mobrob_util/ME439WheelAngles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439WheelAngles)))
  "Returns string type for a message object of type 'ME439WheelAngles"
  "mobrob_util/ME439WheelAngles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439WheelAngles>)))
  "Returns md5sum for a message object of type '<ME439WheelAngles>"
  "8e9b84200a184649eafa090ea3ef1869")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439WheelAngles)))
  "Returns md5sum for a message object of type 'ME439WheelAngles"
  "8e9b84200a184649eafa090ea3ef1869")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439WheelAngles>)))
  "Returns full string definition for message of type '<ME439WheelAngles>"
  (cl:format cl:nil "float64 ang_left~%float64 ang_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439WheelAngles)))
  "Returns full string definition for message of type 'ME439WheelAngles"
  (cl:format cl:nil "float64 ang_left~%float64 ang_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439WheelAngles>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439WheelAngles>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439WheelAngles
    (cl:cons ':ang_left (ang_left msg))
    (cl:cons ':ang_right (ang_right msg))
))
