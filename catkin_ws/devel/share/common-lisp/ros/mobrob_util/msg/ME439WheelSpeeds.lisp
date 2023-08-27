; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439WheelSpeeds.msg.html

(cl:defclass <ME439WheelSpeeds> (roslisp-msg-protocol:ros-message)
  ((v_left
    :reader v_left
    :initarg :v_left
    :type cl:float
    :initform 0.0)
   (v_right
    :reader v_right
    :initarg :v_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass ME439WheelSpeeds (<ME439WheelSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439WheelSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439WheelSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439WheelSpeeds> is deprecated: use mobrob_util-msg:ME439WheelSpeeds instead.")))

(cl:ensure-generic-function 'v_left-val :lambda-list '(m))
(cl:defmethod v_left-val ((m <ME439WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:v_left-val is deprecated.  Use mobrob_util-msg:v_left instead.")
  (v_left m))

(cl:ensure-generic-function 'v_right-val :lambda-list '(m))
(cl:defmethod v_right-val ((m <ME439WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:v_right-val is deprecated.  Use mobrob_util-msg:v_right instead.")
  (v_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439WheelSpeeds>) ostream)
  "Serializes a message object of type '<ME439WheelSpeeds>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439WheelSpeeds>) istream)
  "Deserializes a message object of type '<ME439WheelSpeeds>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439WheelSpeeds>)))
  "Returns string type for a message object of type '<ME439WheelSpeeds>"
  "mobrob_util/ME439WheelSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439WheelSpeeds)))
  "Returns string type for a message object of type 'ME439WheelSpeeds"
  "mobrob_util/ME439WheelSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439WheelSpeeds>)))
  "Returns md5sum for a message object of type '<ME439WheelSpeeds>"
  "db3ff789940b54ebf58420f0dabd5157")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439WheelSpeeds)))
  "Returns md5sum for a message object of type 'ME439WheelSpeeds"
  "db3ff789940b54ebf58420f0dabd5157")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439WheelSpeeds>)))
  "Returns full string definition for message of type '<ME439WheelSpeeds>"
  (cl:format cl:nil "float32 v_left~%float32 v_right~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439WheelSpeeds)))
  "Returns full string definition for message of type 'ME439WheelSpeeds"
  (cl:format cl:nil "float32 v_left~%float32 v_right~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439WheelSpeeds>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439WheelSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439WheelSpeeds
    (cl:cons ':v_left (v_left msg))
    (cl:cons ':v_right (v_right msg))
))
