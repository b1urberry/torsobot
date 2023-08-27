; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439SensorsRaw.msg.html

(cl:defclass <ME439SensorsRaw> (roslisp-msg-protocol:ros-message)
  ((e0
    :reader e0
    :initarg :e0
    :type cl:integer
    :initform 0)
   (e1
    :reader e1
    :initarg :e1
    :type cl:integer
    :initform 0)
   (a0
    :reader a0
    :initarg :a0
    :type cl:fixnum
    :initform 0)
   (a1
    :reader a1
    :initarg :a1
    :type cl:fixnum
    :initform 0)
   (a2
    :reader a2
    :initarg :a2
    :type cl:fixnum
    :initform 0)
   (a3
    :reader a3
    :initarg :a3
    :type cl:fixnum
    :initform 0)
   (a4
    :reader a4
    :initarg :a4
    :type cl:fixnum
    :initform 0)
   (a5
    :reader a5
    :initarg :a5
    :type cl:fixnum
    :initform 0)
   (u0
    :reader u0
    :initarg :u0
    :type cl:integer
    :initform 0)
   (u1
    :reader u1
    :initarg :u1
    :type cl:integer
    :initform 0)
   (u2
    :reader u2
    :initarg :u2
    :type cl:integer
    :initform 0)
   (t
    :reader t
    :initarg :t
    :type cl:real
    :initform 0))
)

(cl:defclass ME439SensorsRaw (<ME439SensorsRaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439SensorsRaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439SensorsRaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439SensorsRaw> is deprecated: use mobrob_util-msg:ME439SensorsRaw instead.")))

(cl:ensure-generic-function 'e0-val :lambda-list '(m))
(cl:defmethod e0-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e0-val is deprecated.  Use mobrob_util-msg:e0 instead.")
  (e0 m))

(cl:ensure-generic-function 'e1-val :lambda-list '(m))
(cl:defmethod e1-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e1-val is deprecated.  Use mobrob_util-msg:e1 instead.")
  (e1 m))

(cl:ensure-generic-function 'a0-val :lambda-list '(m))
(cl:defmethod a0-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a0-val is deprecated.  Use mobrob_util-msg:a0 instead.")
  (a0 m))

(cl:ensure-generic-function 'a1-val :lambda-list '(m))
(cl:defmethod a1-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a1-val is deprecated.  Use mobrob_util-msg:a1 instead.")
  (a1 m))

(cl:ensure-generic-function 'a2-val :lambda-list '(m))
(cl:defmethod a2-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a2-val is deprecated.  Use mobrob_util-msg:a2 instead.")
  (a2 m))

(cl:ensure-generic-function 'a3-val :lambda-list '(m))
(cl:defmethod a3-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a3-val is deprecated.  Use mobrob_util-msg:a3 instead.")
  (a3 m))

(cl:ensure-generic-function 'a4-val :lambda-list '(m))
(cl:defmethod a4-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a4-val is deprecated.  Use mobrob_util-msg:a4 instead.")
  (a4 m))

(cl:ensure-generic-function 'a5-val :lambda-list '(m))
(cl:defmethod a5-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a5-val is deprecated.  Use mobrob_util-msg:a5 instead.")
  (a5 m))

(cl:ensure-generic-function 'u0-val :lambda-list '(m))
(cl:defmethod u0-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u0-val is deprecated.  Use mobrob_util-msg:u0 instead.")
  (u0 m))

(cl:ensure-generic-function 'u1-val :lambda-list '(m))
(cl:defmethod u1-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u1-val is deprecated.  Use mobrob_util-msg:u1 instead.")
  (u1 m))

(cl:ensure-generic-function 'u2-val :lambda-list '(m))
(cl:defmethod u2-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u2-val is deprecated.  Use mobrob_util-msg:u2 instead.")
  (u2 m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <ME439SensorsRaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:t-val is deprecated.  Use mobrob_util-msg:t instead.")
  (t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439SensorsRaw>) ostream)
  "Serializes a message object of type '<ME439SensorsRaw>"
  (cl:let* ((signed (cl:slot-value msg 'e0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'e1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a3)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a4)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'a5)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'u0)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'u1)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'u2)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__sec (cl:floor (cl:slot-value msg 't)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 't) (cl:floor (cl:slot-value msg 't)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439SensorsRaw>) istream)
  "Deserializes a message object of type '<ME439SensorsRaw>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'e0) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'e1) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a0) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a1) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a2) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a3) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a4) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a5) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'u0) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'u1) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'u2) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 't) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439SensorsRaw>)))
  "Returns string type for a message object of type '<ME439SensorsRaw>"
  "mobrob_util/ME439SensorsRaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439SensorsRaw)))
  "Returns string type for a message object of type 'ME439SensorsRaw"
  "mobrob_util/ME439SensorsRaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439SensorsRaw>)))
  "Returns md5sum for a message object of type '<ME439SensorsRaw>"
  "e832476c98d8752e8bb419b8adb5fe2a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439SensorsRaw)))
  "Returns md5sum for a message object of type 'ME439SensorsRaw"
  "e832476c98d8752e8bb419b8adb5fe2a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439SensorsRaw>)))
  "Returns full string definition for message of type '<ME439SensorsRaw>"
  (cl:format cl:nil "int64 e0~%int64 e1~%int16 a0~%int16 a1~%int16 a2~%int16 a3~%int16 a4~%int16 a5~%int32 u0~%int32 u1~%int32 u2~%time t~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439SensorsRaw)))
  "Returns full string definition for message of type 'ME439SensorsRaw"
  (cl:format cl:nil "int64 e0~%int64 e1~%int16 a0~%int16 a1~%int16 a2~%int16 a3~%int16 a4~%int16 a5~%int32 u0~%int32 u1~%int32 u2~%time t~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439SensorsRaw>))
  (cl:+ 0
     8
     8
     2
     2
     2
     2
     2
     2
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439SensorsRaw>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439SensorsRaw
    (cl:cons ':e0 (e0 msg))
    (cl:cons ':e1 (e1 msg))
    (cl:cons ':a0 (a0 msg))
    (cl:cons ':a1 (a1 msg))
    (cl:cons ':a2 (a2 msg))
    (cl:cons ':a3 (a3 msg))
    (cl:cons ':a4 (a4 msg))
    (cl:cons ':a5 (a5 msg))
    (cl:cons ':u0 (u0 msg))
    (cl:cons ':u1 (u1 msg))
    (cl:cons ':u2 (u2 msg))
    (cl:cons ':t (t msg))
))
