; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439MotorCommands.msg.html

(cl:defclass <ME439MotorCommands> (roslisp-msg-protocol:ros-message)
  ((cmd0
    :reader cmd0
    :initarg :cmd0
    :type cl:fixnum
    :initform 0)
   (cmd1
    :reader cmd1
    :initarg :cmd1
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ME439MotorCommands (<ME439MotorCommands>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439MotorCommands>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439MotorCommands)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439MotorCommands> is deprecated: use mobrob_util-msg:ME439MotorCommands instead.")))

(cl:ensure-generic-function 'cmd0-val :lambda-list '(m))
(cl:defmethod cmd0-val ((m <ME439MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:cmd0-val is deprecated.  Use mobrob_util-msg:cmd0 instead.")
  (cmd0 m))

(cl:ensure-generic-function 'cmd1-val :lambda-list '(m))
(cl:defmethod cmd1-val ((m <ME439MotorCommands>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:cmd1-val is deprecated.  Use mobrob_util-msg:cmd1 instead.")
  (cmd1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439MotorCommands>) ostream)
  "Serializes a message object of type '<ME439MotorCommands>"
  (cl:let* ((signed (cl:slot-value msg 'cmd0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cmd1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439MotorCommands>) istream)
  "Deserializes a message object of type '<ME439MotorCommands>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd0) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439MotorCommands>)))
  "Returns string type for a message object of type '<ME439MotorCommands>"
  "mobrob_util/ME439MotorCommands")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439MotorCommands)))
  "Returns string type for a message object of type 'ME439MotorCommands"
  "mobrob_util/ME439MotorCommands")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439MotorCommands>)))
  "Returns md5sum for a message object of type '<ME439MotorCommands>"
  "853bb91b9c3f09306e8c71787dc35bdf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439MotorCommands)))
  "Returns md5sum for a message object of type 'ME439MotorCommands"
  "853bb91b9c3f09306e8c71787dc35bdf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439MotorCommands>)))
  "Returns full string definition for message of type '<ME439MotorCommands>"
  (cl:format cl:nil "int16 cmd0~%int16 cmd1~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439MotorCommands)))
  "Returns full string definition for message of type 'ME439MotorCommands"
  (cl:format cl:nil "int16 cmd0~%int16 cmd1~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439MotorCommands>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439MotorCommands>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439MotorCommands
    (cl:cons ':cmd0 (cmd0 msg))
    (cl:cons ':cmd1 (cmd1 msg))
))
