; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439WheelDisplacements.msg.html

(cl:defclass <ME439WheelDisplacements> (roslisp-msg-protocol:ros-message)
  ((d_left
    :reader d_left
    :initarg :d_left
    :type cl:float
    :initform 0.0)
   (d_right
    :reader d_right
    :initarg :d_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass ME439WheelDisplacements (<ME439WheelDisplacements>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439WheelDisplacements>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439WheelDisplacements)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439WheelDisplacements> is deprecated: use mobrob_util-msg:ME439WheelDisplacements instead.")))

(cl:ensure-generic-function 'd_left-val :lambda-list '(m))
(cl:defmethod d_left-val ((m <ME439WheelDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:d_left-val is deprecated.  Use mobrob_util-msg:d_left instead.")
  (d_left m))

(cl:ensure-generic-function 'd_right-val :lambda-list '(m))
(cl:defmethod d_right-val ((m <ME439WheelDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:d_right-val is deprecated.  Use mobrob_util-msg:d_right instead.")
  (d_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439WheelDisplacements>) ostream)
  "Serializes a message object of type '<ME439WheelDisplacements>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'd_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'd_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439WheelDisplacements>) istream)
  "Deserializes a message object of type '<ME439WheelDisplacements>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439WheelDisplacements>)))
  "Returns string type for a message object of type '<ME439WheelDisplacements>"
  "mobrob_util/ME439WheelDisplacements")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439WheelDisplacements)))
  "Returns string type for a message object of type 'ME439WheelDisplacements"
  "mobrob_util/ME439WheelDisplacements")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439WheelDisplacements>)))
  "Returns md5sum for a message object of type '<ME439WheelDisplacements>"
  "0c76547e5c694fd23ef9ed1874ecca4a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439WheelDisplacements)))
  "Returns md5sum for a message object of type 'ME439WheelDisplacements"
  "0c76547e5c694fd23ef9ed1874ecca4a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439WheelDisplacements>)))
  "Returns full string definition for message of type '<ME439WheelDisplacements>"
  (cl:format cl:nil "float64 d_left~%float64 d_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439WheelDisplacements)))
  "Returns full string definition for message of type 'ME439WheelDisplacements"
  (cl:format cl:nil "float64 d_left~%float64 d_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439WheelDisplacements>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439WheelDisplacements>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439WheelDisplacements
    (cl:cons ':d_left (d_left msg))
    (cl:cons ':d_right (d_right msg))
))
