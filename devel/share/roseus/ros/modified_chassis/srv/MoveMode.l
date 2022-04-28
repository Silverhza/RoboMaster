;; Auto-generated. Do not edit!


(when (boundp 'modified_chassis::MoveMode)
  (if (not (find-package "MODIFIED_CHASSIS"))
    (make-package "MODIFIED_CHASSIS"))
  (shadow 'MoveMode (find-package "MODIFIED_CHASSIS")))
(unless (find-package "MODIFIED_CHASSIS::MOVEMODE")
  (make-package "MODIFIED_CHASSIS::MOVEMODE"))
(unless (find-package "MODIFIED_CHASSIS::MOVEMODEREQUEST")
  (make-package "MODIFIED_CHASSIS::MOVEMODEREQUEST"))
(unless (find-package "MODIFIED_CHASSIS::MOVEMODERESPONSE")
  (make-package "MODIFIED_CHASSIS::MOVEMODERESPONSE"))

(in-package "ROS")





(defclass modified_chassis::MoveModeRequest
  :super ros::object
  :slots (_mode ))

(defmethod modified_chassis::MoveModeRequest
  (:init
   (&key
    ((:mode __mode) nil)
    )
   (send-super :init)
   (setq _mode __mode)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; bool _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _mode
       (if _mode (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _mode
     (setq _mode (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass modified_chassis::MoveModeResponse
  :super ros::object
  :slots (_received ))

(defmethod modified_chassis::MoveModeResponse
  (:init
   (&key
    ((:received __received) nil)
    )
   (send-super :init)
   (setq _received __received)
   self)
  (:received
   (&optional __received)
   (if __received (setq _received __received)) _received)
  (:serialization-length
   ()
   (+
    ;; bool _received
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _received
       (if _received (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _received
     (setq _received (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass modified_chassis::MoveMode
  :super ros::object
  :slots ())

(setf (get modified_chassis::MoveMode :md5sum-) "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(setf (get modified_chassis::MoveMode :datatype-) "modified_chassis/MoveMode")
(setf (get modified_chassis::MoveMode :request) modified_chassis::MoveModeRequest)
(setf (get modified_chassis::MoveMode :response) modified_chassis::MoveModeResponse)

(defmethod modified_chassis::MoveModeRequest
  (:response () (instance modified_chassis::MoveModeResponse :init)))

(setf (get modified_chassis::MoveModeRequest :md5sum-) "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(setf (get modified_chassis::MoveModeRequest :datatype-) "modified_chassis/MoveModeRequest")
(setf (get modified_chassis::MoveModeRequest :definition-)
      "bool mode
---
bool received

")

(setf (get modified_chassis::MoveModeResponse :md5sum-) "e5d38bc3f9d0bc1f71af55e7a5ccf455")
(setf (get modified_chassis::MoveModeResponse :datatype-) "modified_chassis/MoveModeResponse")
(setf (get modified_chassis::MoveModeResponse :definition-)
      "bool mode
---
bool received

")



(provide :modified_chassis/MoveMode "e5d38bc3f9d0bc1f71af55e7a5ccf455")


