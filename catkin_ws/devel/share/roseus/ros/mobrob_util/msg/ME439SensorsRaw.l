;; Auto-generated. Do not edit!


(when (boundp 'mobrob_util::ME439SensorsRaw)
  (if (not (find-package "MOBROB_UTIL"))
    (make-package "MOBROB_UTIL"))
  (shadow 'ME439SensorsRaw (find-package "MOBROB_UTIL")))
(unless (find-package "MOBROB_UTIL::ME439SENSORSRAW")
  (make-package "MOBROB_UTIL::ME439SENSORSRAW"))

(in-package "ROS")
;;//! \htmlinclude ME439SensorsRaw.msg.html


(defclass mobrob_util::ME439SensorsRaw
  :super ros::object
  :slots (_e0 _e1 _a0 _a1 _a2 _a3 _a4 _a5 _u0 _u1 _u2 _t ))

(defmethod mobrob_util::ME439SensorsRaw
  (:init
   (&key
    ((:e0 __e0) 0)
    ((:e1 __e1) 0)
    ((:a0 __a0) 0)
    ((:a1 __a1) 0)
    ((:a2 __a2) 0)
    ((:a3 __a3) 0)
    ((:a4 __a4) 0)
    ((:a5 __a5) 0)
    ((:u0 __u0) 0)
    ((:u1 __u1) 0)
    ((:u2 __u2) 0)
    ((:t __t) (instance ros::time :init))
    )
   (send-super :init)
   (setq _e0 (round __e0))
   (setq _e1 (round __e1))
   (setq _a0 (round __a0))
   (setq _a1 (round __a1))
   (setq _a2 (round __a2))
   (setq _a3 (round __a3))
   (setq _a4 (round __a4))
   (setq _a5 (round __a5))
   (setq _u0 (round __u0))
   (setq _u1 (round __u1))
   (setq _u2 (round __u2))
   (setq _t __t)
   self)
  (:e0
   (&optional __e0)
   (if __e0 (setq _e0 __e0)) _e0)
  (:e1
   (&optional __e1)
   (if __e1 (setq _e1 __e1)) _e1)
  (:a0
   (&optional __a0)
   (if __a0 (setq _a0 __a0)) _a0)
  (:a1
   (&optional __a1)
   (if __a1 (setq _a1 __a1)) _a1)
  (:a2
   (&optional __a2)
   (if __a2 (setq _a2 __a2)) _a2)
  (:a3
   (&optional __a3)
   (if __a3 (setq _a3 __a3)) _a3)
  (:a4
   (&optional __a4)
   (if __a4 (setq _a4 __a4)) _a4)
  (:a5
   (&optional __a5)
   (if __a5 (setq _a5 __a5)) _a5)
  (:u0
   (&optional __u0)
   (if __u0 (setq _u0 __u0)) _u0)
  (:u1
   (&optional __u1)
   (if __u1 (setq _u1 __u1)) _u1)
  (:u2
   (&optional __u2)
   (if __u2 (setq _u2 __u2)) _u2)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; int64 _e0
    8
    ;; int64 _e1
    8
    ;; int16 _a0
    2
    ;; int16 _a1
    2
    ;; int16 _a2
    2
    ;; int16 _a3
    2
    ;; int16 _a4
    2
    ;; int16 _a5
    2
    ;; int32 _u0
    4
    ;; int32 _u1
    4
    ;; int32 _u2
    4
    ;; time _t
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _e0
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _e0 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _e0) (= (length (_e0 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_e0 . bv) 0) 0) s)
              (write-long (ash (elt (_e0 . bv) 1) -1) s))
             ((and (class _e0) (= (length (_e0 . bv)) 1)) ;; big1
              (write-long (elt (_e0 . bv) 0) s)
              (write-long (if (>= _e0 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _e0 s)(write-long (if (>= _e0 0) 0 #xffffffff) s)))
     ;; int64 _e1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _e1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _e1) (= (length (_e1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_e1 . bv) 0) 0) s)
              (write-long (ash (elt (_e1 . bv) 1) -1) s))
             ((and (class _e1) (= (length (_e1 . bv)) 1)) ;; big1
              (write-long (elt (_e1 . bv) 0) s)
              (write-long (if (>= _e1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _e1 s)(write-long (if (>= _e1 0) 0 #xffffffff) s)))
     ;; int16 _a0
       (write-word _a0 s)
     ;; int16 _a1
       (write-word _a1 s)
     ;; int16 _a2
       (write-word _a2 s)
     ;; int16 _a3
       (write-word _a3 s)
     ;; int16 _a4
       (write-word _a4 s)
     ;; int16 _a5
       (write-word _a5 s)
     ;; int32 _u0
       (write-long _u0 s)
     ;; int32 _u1
       (write-long _u1 s)
     ;; int32 _u2
       (write-long _u2 s)
     ;; time _t
       (write-long (send _t :sec) s) (write-long (send _t :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _e0
#+(or :alpha :irix6 :x86_64)
      (setf _e0 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _e0 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _e1
#+(or :alpha :irix6 :x86_64)
      (setf _e1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _e1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int16 _a0
     (setq _a0 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a1
     (setq _a1 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a2
     (setq _a2 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a3
     (setq _a3 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a4
     (setq _a4 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _a5
     (setq _a5 (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int32 _u0
     (setq _u0 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u1
     (setq _u1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _u2
     (setq _u2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _t
     (send _t :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _t :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mobrob_util::ME439SensorsRaw :md5sum-) "e832476c98d8752e8bb419b8adb5fe2a")
(setf (get mobrob_util::ME439SensorsRaw :datatype-) "mobrob_util/ME439SensorsRaw")
(setf (get mobrob_util::ME439SensorsRaw :definition-)
      "int64 e0
int64 e1
int16 a0
int16 a1
int16 a2
int16 a3
int16 a4
int16 a5
int32 u0
int32 u1
int32 u2
time t
")



(provide :mobrob_util/ME439SensorsRaw "e832476c98d8752e8bb419b8adb5fe2a")


