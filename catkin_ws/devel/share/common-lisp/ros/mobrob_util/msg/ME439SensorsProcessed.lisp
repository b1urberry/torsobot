; Auto-generated. Do not edit!


(cl:in-package mobrob_util-msg)


;//! \htmlinclude ME439SensorsProcessed.msg.html

(cl:defclass <ME439SensorsProcessed> (roslisp-msg-protocol:ros-message)
  ((e0radians
    :reader e0radians
    :initarg :e0radians
    :type cl:float
    :initform 0.0)
   (e1radians
    :reader e1radians
    :initarg :e1radians
    :type cl:float
    :initform 0.0)
   (e0meters
    :reader e0meters
    :initarg :e0meters
    :type cl:float
    :initform 0.0)
   (e1meters
    :reader e1meters
    :initarg :e1meters
    :type cl:float
    :initform 0.0)
   (e0radpersec
    :reader e0radpersec
    :initarg :e0radpersec
    :type cl:float
    :initform 0.0)
   (e1radpersec
    :reader e1radpersec
    :initarg :e1radpersec
    :type cl:float
    :initform 0.0)
   (e0meterspersec
    :reader e0meterspersec
    :initarg :e0meterspersec
    :type cl:float
    :initform 0.0)
   (e1meterspersec
    :reader e1meterspersec
    :initarg :e1meterspersec
    :type cl:float
    :initform 0.0)
   (a0
    :reader a0
    :initarg :a0
    :type cl:float
    :initform 0.0)
   (a1
    :reader a1
    :initarg :a1
    :type cl:float
    :initform 0.0)
   (a2
    :reader a2
    :initarg :a2
    :type cl:float
    :initform 0.0)
   (a3
    :reader a3
    :initarg :a3
    :type cl:float
    :initform 0.0)
   (a4
    :reader a4
    :initarg :a4
    :type cl:float
    :initform 0.0)
   (a5
    :reader a5
    :initarg :a5
    :type cl:float
    :initform 0.0)
   (u0meters
    :reader u0meters
    :initarg :u0meters
    :type cl:float
    :initform 0.0)
   (u1meters
    :reader u1meters
    :initarg :u1meters
    :type cl:float
    :initform 0.0)
   (u2meters
    :reader u2meters
    :initarg :u2meters
    :type cl:float
    :initform 0.0)
   (dt
    :reader dt
    :initarg :dt
    :type cl:real
    :initform 0))
)

(cl:defclass ME439SensorsProcessed (<ME439SensorsProcessed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ME439SensorsProcessed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ME439SensorsProcessed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mobrob_util-msg:<ME439SensorsProcessed> is deprecated: use mobrob_util-msg:ME439SensorsProcessed instead.")))

(cl:ensure-generic-function 'e0radians-val :lambda-list '(m))
(cl:defmethod e0radians-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e0radians-val is deprecated.  Use mobrob_util-msg:e0radians instead.")
  (e0radians m))

(cl:ensure-generic-function 'e1radians-val :lambda-list '(m))
(cl:defmethod e1radians-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e1radians-val is deprecated.  Use mobrob_util-msg:e1radians instead.")
  (e1radians m))

(cl:ensure-generic-function 'e0meters-val :lambda-list '(m))
(cl:defmethod e0meters-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e0meters-val is deprecated.  Use mobrob_util-msg:e0meters instead.")
  (e0meters m))

(cl:ensure-generic-function 'e1meters-val :lambda-list '(m))
(cl:defmethod e1meters-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e1meters-val is deprecated.  Use mobrob_util-msg:e1meters instead.")
  (e1meters m))

(cl:ensure-generic-function 'e0radpersec-val :lambda-list '(m))
(cl:defmethod e0radpersec-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e0radpersec-val is deprecated.  Use mobrob_util-msg:e0radpersec instead.")
  (e0radpersec m))

(cl:ensure-generic-function 'e1radpersec-val :lambda-list '(m))
(cl:defmethod e1radpersec-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e1radpersec-val is deprecated.  Use mobrob_util-msg:e1radpersec instead.")
  (e1radpersec m))

(cl:ensure-generic-function 'e0meterspersec-val :lambda-list '(m))
(cl:defmethod e0meterspersec-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e0meterspersec-val is deprecated.  Use mobrob_util-msg:e0meterspersec instead.")
  (e0meterspersec m))

(cl:ensure-generic-function 'e1meterspersec-val :lambda-list '(m))
(cl:defmethod e1meterspersec-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:e1meterspersec-val is deprecated.  Use mobrob_util-msg:e1meterspersec instead.")
  (e1meterspersec m))

(cl:ensure-generic-function 'a0-val :lambda-list '(m))
(cl:defmethod a0-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a0-val is deprecated.  Use mobrob_util-msg:a0 instead.")
  (a0 m))

(cl:ensure-generic-function 'a1-val :lambda-list '(m))
(cl:defmethod a1-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a1-val is deprecated.  Use mobrob_util-msg:a1 instead.")
  (a1 m))

(cl:ensure-generic-function 'a2-val :lambda-list '(m))
(cl:defmethod a2-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a2-val is deprecated.  Use mobrob_util-msg:a2 instead.")
  (a2 m))

(cl:ensure-generic-function 'a3-val :lambda-list '(m))
(cl:defmethod a3-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a3-val is deprecated.  Use mobrob_util-msg:a3 instead.")
  (a3 m))

(cl:ensure-generic-function 'a4-val :lambda-list '(m))
(cl:defmethod a4-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a4-val is deprecated.  Use mobrob_util-msg:a4 instead.")
  (a4 m))

(cl:ensure-generic-function 'a5-val :lambda-list '(m))
(cl:defmethod a5-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:a5-val is deprecated.  Use mobrob_util-msg:a5 instead.")
  (a5 m))

(cl:ensure-generic-function 'u0meters-val :lambda-list '(m))
(cl:defmethod u0meters-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u0meters-val is deprecated.  Use mobrob_util-msg:u0meters instead.")
  (u0meters m))

(cl:ensure-generic-function 'u1meters-val :lambda-list '(m))
(cl:defmethod u1meters-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u1meters-val is deprecated.  Use mobrob_util-msg:u1meters instead.")
  (u1meters m))

(cl:ensure-generic-function 'u2meters-val :lambda-list '(m))
(cl:defmethod u2meters-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:u2meters-val is deprecated.  Use mobrob_util-msg:u2meters instead.")
  (u2meters m))

(cl:ensure-generic-function 'dt-val :lambda-list '(m))
(cl:defmethod dt-val ((m <ME439SensorsProcessed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mobrob_util-msg:dt-val is deprecated.  Use mobrob_util-msg:dt instead.")
  (dt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ME439SensorsProcessed>) ostream)
  "Serializes a message object of type '<ME439SensorsProcessed>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e0radians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e1radians))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e0meters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e1meters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e0radpersec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e1radpersec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e0meterspersec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'e1meterspersec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'a5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u0meters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u1meters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u2meters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'dt)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'dt) (cl:floor (cl:slot-value msg 'dt)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ME439SensorsProcessed>) istream)
  "Deserializes a message object of type '<ME439SensorsProcessed>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e0radians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e1radians) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e0meters) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e1meters) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e0radpersec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e1radpersec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e0meterspersec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'e1meterspersec) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u0meters) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u1meters) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u2meters) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dt) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ME439SensorsProcessed>)))
  "Returns string type for a message object of type '<ME439SensorsProcessed>"
  "mobrob_util/ME439SensorsProcessed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ME439SensorsProcessed)))
  "Returns string type for a message object of type 'ME439SensorsProcessed"
  "mobrob_util/ME439SensorsProcessed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ME439SensorsProcessed>)))
  "Returns md5sum for a message object of type '<ME439SensorsProcessed>"
  "44afd1f89023a5ad6799a84d47e3ad41")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ME439SensorsProcessed)))
  "Returns md5sum for a message object of type 'ME439SensorsProcessed"
  "44afd1f89023a5ad6799a84d47e3ad41")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ME439SensorsProcessed>)))
  "Returns full string definition for message of type '<ME439SensorsProcessed>"
  (cl:format cl:nil "float32 e0radians~%float32 e1radians~%float32 e0meters~%float32 e1meters~%float32 e0radpersec~%float32 e1radpersec~%float32 e0meterspersec~%float32 e1meterspersec~%float32 a0~%float32 a1~%float32 a2~%float32 a3~%float32 a4~%float32 a5~%float32 u0meters~%float32 u1meters~%float32 u2meters~%duration dt~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ME439SensorsProcessed)))
  "Returns full string definition for message of type 'ME439SensorsProcessed"
  (cl:format cl:nil "float32 e0radians~%float32 e1radians~%float32 e0meters~%float32 e1meters~%float32 e0radpersec~%float32 e1radpersec~%float32 e0meterspersec~%float32 e1meterspersec~%float32 a0~%float32 a1~%float32 a2~%float32 a3~%float32 a4~%float32 a5~%float32 u0meters~%float32 u1meters~%float32 u2meters~%duration dt~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ME439SensorsProcessed>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ME439SensorsProcessed>))
  "Converts a ROS message object to a list"
  (cl:list 'ME439SensorsProcessed
    (cl:cons ':e0radians (e0radians msg))
    (cl:cons ':e1radians (e1radians msg))
    (cl:cons ':e0meters (e0meters msg))
    (cl:cons ':e1meters (e1meters msg))
    (cl:cons ':e0radpersec (e0radpersec msg))
    (cl:cons ':e1radpersec (e1radpersec msg))
    (cl:cons ':e0meterspersec (e0meterspersec msg))
    (cl:cons ':e1meterspersec (e1meterspersec msg))
    (cl:cons ':a0 (a0 msg))
    (cl:cons ':a1 (a1 msg))
    (cl:cons ':a2 (a2 msg))
    (cl:cons ':a3 (a3 msg))
    (cl:cons ':a4 (a4 msg))
    (cl:cons ':a5 (a5 msg))
    (cl:cons ':u0meters (u0meters msg))
    (cl:cons ':u1meters (u1meters msg))
    (cl:cons ':u2meters (u2meters msg))
    (cl:cons ':dt (dt msg))
))
