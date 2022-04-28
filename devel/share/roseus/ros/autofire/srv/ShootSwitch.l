;; Auto-generated. Do not edit!


(when (boundp 'autofire::ShootSwitch)
  (if (not (find-package "AUTOFIRE"))
    (make-package "AUTOFIRE"))
  (shadow 'ShootSwitch (find-package "AUTOFIRE")))
(unless (find-package "AUTOFIRE::SHOOTSWITCH")
  (make-package "AUTOFIRE::SHOOTSWITCH"))
(unless (find-package "AUTOFIRE::SHOOTSWITCHREQUEST")
  (make-package "AUTOFIRE::SHOOTSWITCHREQUEST"))
(unless (find-package "AUTOFIRE::SHOOTSWITCHRESPONSE")
  (make-package "AUTOFIRE::SHOOTSWITCHRESPONSE"))

(in-package "ROS")





(defclass autofire::ShootSwitchRequest
  :super ros::object
  :slots (_enableShoot ))

(defmethod autofire::ShootSwitchRequest
  (:init
   (&key
    ((:enableShoot __enableShoot) nil)
    )
   (send-super :init)
   (setq _enableShoot __enableShoot)
   self)
  (:enableShoot
   (&optional __enableShoot)
   (if __enableShoot (setq _enableShoot __enableShoot)) _enableShoot)
  (:serialization-length
   ()
   (+
    ;; bool _enableShoot
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enableShoot
       (if _enableShoot (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enableShoot
     (setq _enableShoot (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass autofire::ShootSwitchResponse
  :super ros::object
  :slots (_status ))

(defmethod autofire::ShootSwitchResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass autofire::ShootSwitch
  :super ros::object
  :slots ())

(setf (get autofire::ShootSwitch :md5sum-) "5d377b7485c09603dc7fd050a7504601")
(setf (get autofire::ShootSwitch :datatype-) "autofire/ShootSwitch")
(setf (get autofire::ShootSwitch :request) autofire::ShootSwitchRequest)
(setf (get autofire::ShootSwitch :response) autofire::ShootSwitchResponse)

(defmethod autofire::ShootSwitchRequest
  (:response () (instance autofire::ShootSwitchResponse :init)))

(setf (get autofire::ShootSwitchRequest :md5sum-) "5d377b7485c09603dc7fd050a7504601")
(setf (get autofire::ShootSwitchRequest :datatype-) "autofire/ShootSwitchRequest")
(setf (get autofire::ShootSwitchRequest :definition-)
      "bool enableShoot
---
bool status

")

(setf (get autofire::ShootSwitchResponse :md5sum-) "5d377b7485c09603dc7fd050a7504601")
(setf (get autofire::ShootSwitchResponse :datatype-) "autofire/ShootSwitchResponse")
(setf (get autofire::ShootSwitchResponse :definition-)
      "bool enableShoot
---
bool status

")



(provide :autofire/ShootSwitch "5d377b7485c09603dc7fd050a7504601")


