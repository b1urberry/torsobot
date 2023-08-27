; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439PathSpecs.msg.html

(cl:defclass <ME439PathSpecs> (roslisp-msg-protocol:ros-message)
  ((x0
    :reader x0
    :initarg :x0
    :type cl:float
    :initform 0.0)
   (y0
    :reader y0
    :initarg :y0
    :type cl:float
    :initform 0.0)
   (theta0
    :reader theta0
    :initarg :theta0
    :type cl:float
    :initform 0.0)
   (Radius
    :reader Radius
    :initarg :Radius
    :type cl:float
    :initform 0.0)
   (Length
    :reader Length
    :initarg :Length
    :type cl:float
    :initform 0.0))
)

(cl:defclass ME439PathSpecs (<ME439PathSpecs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439PathSpecs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439PathSpecs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439PathSpecs> is deprecated: use mobrob_util-msg:ME439PathSpecs instead.")))

(cl:ensure-generic-function 'x0-val :lambda-list '(m))
(cl:defmethod x0-val ((m <ME439PathSpecs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:x0-val is deprecated.  Use mobrob_util-msg:x0 instead.")
  (x0 m))

(cl:ensure-generic-function 'y0-val :lambda-list '(m))
(cl:defmethod y0-val ((m <ME439PathSpecs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:y0-val is deprecated.  Use mobrob_util-msg:y0 instead.")
  (y0 m))

(cl:ensure-generic-function 'theta0-val :lambda-list '(m))
(cl:defmethod theta0-val ((m <ME439PathSpecs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:theta0-val is deprecated.  Use mobrob_util-msg:theta0 instead.")
  (theta0 m))

(cl:ensure-generic-function 'Radius-val :lambda-list '(m))
(cl:defmethod Radius-val ((m <ME439PathSpecs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:Radius-val is deprecated.  Use mobrob_util-msg:Radius instead.")
  (Radius m))

(cl:ensure-generic-function 'Length-val :lambda-list '(m))
(cl:defmethod Length-val ((m <ME439PathSpecs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:Length-val is deprecated.  Use mobrob_util-msg:Length instead.")
  (Length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439PathSpecs>) ostream)
  "Serializes a message object of type '<ME439PathSpecs>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439PathSpecs>) istream)
  "Deserializes a message object of type '<ME439PathSpecs>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Radius) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Length) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439PathSpecs>)))
  "Returns string type for a message object of type '<ME439PathSpecs>"
  "mobrob_util/ME439PathSpecs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439PathSpecs)))
  "Returns string type for a message object of type 'ME439PathSpecs"
  "mobrob_util/ME439PathSpecs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439PathSpecs>)))
  "Returns md5sum for a message object of type '<ME439PathSpecs>"
  "278b13eb1b2cd154f8602d6118f2c62d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439PathSpecs)))
  "Returns md5sum for a message object of type 'ME439PathSpecs"
  "278b13eb1b2cd154f8602d6118f2c62d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439PathSpecs>)))
  "Returns full string definition for message of type '<ME439PathSpecs>"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 theta0~%float32 Radius~%float32 Length~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439PathSpecs)))
  "Returns full string definition for message of type 'ME439PathSpecs"
  (cl:format cl:nil "float32 x0~%float32 y0~%float32 theta0~%float32 Radius~%float32 Length~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439PathSpecs>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439PathSpecs>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439PathSpecs
    (cl:cons ':x0 (x0 msg))
    (cl:cons ':y0 (y0 msg))
    (cl:cons ':theta0 (theta0 msg))
    (cl:cons ':Radius (Radius msg))
    (cl:cons ':Length (Length msg))
))
