;; Auto-generated. Do not edit!


(when (boundp 'autofire::Lidar2Enemy)
  (if (not (find-package "AUTOFIRE"))
    (make-package "AUTOFIRE"))
  (shadow 'Lidar2Enemy (find-package "AUTOFIRE")))
(unless (find-package "AUTOFIRE::LIDAR2ENEMY")
  (make-package "AUTOFIRE::LIDAR2ENEMY"))
(unless (find-package "AUTOFIRE::LIDAR2ENEMYREQUEST")
  (make-package "AUTOFIRE::LIDAR2ENEMYREQUEST"))
(unless (find-package "AUTOFIRE::LIDAR2ENEMYRESPONSE")
  (make-package "AUTOFIRE::LIDAR2ENEMYRESPONSE"))

(in-package "ROS")





(defclass autofire::Lidar2EnemyRequest
  :super ros::object
  :slots (_setStatus ))

(defmethod autofire::Lidar2EnemyRequest
  (:init
   (&key
    ((:setStatus __setStatus) nil)
    )
   (send-super :init)
   (setq _setStatus __setStatus)
   self)
  (:setStatus
   (&optional __setStatus)
   (if __setStatus (setq _setStatus __setStatus)) _setStatus)
  (:serialization-length
   ()
   (+
    ;; bool _setStatus
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _setStatus
       (if _setStatus (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _setStatus
     (setq _setStatus (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass autofire::Lidar2EnemyResponse
  :super ros::object
  :slots (_received ))

(defmethod autofire::Lidar2EnemyResponse
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

(defclass autofire::Lidar2Enemy
  :super ros::object
  :slots ())

(setf (get autofire::Lidar2Enemy :md5sum-) "5c20fa677eb90ce8ed6acc054aa17bb1")
(setf (get autofire::Lidar2Enemy :datatype-) "autofire/Lidar2Enemy")
(setf (get autofire::Lidar2Enemy :request) autofire::Lidar2EnemyRequest)
(setf (get autofire::Lidar2Enemy :response) autofire::Lidar2EnemyResponse)

(defmethod autofire::Lidar2EnemyRequest
  (:response () (instance autofire::Lidar2EnemyResponse :init)))

(setf (get autofire::Lidar2EnemyRequest :md5sum-) "5c20fa677eb90ce8ed6acc054aa17bb1")
(setf (get autofire::Lidar2EnemyRequest :datatype-) "autofire/Lidar2EnemyRequest")
(setf (get autofire::Lidar2EnemyRequest :definition-)
      "bool setStatus
---
bool received

")

(setf (get autofire::Lidar2EnemyResponse :md5sum-) "5c20fa677eb90ce8ed6acc054aa17bb1")
(setf (get autofire::Lidar2EnemyResponse :datatype-) "autofire/Lidar2EnemyResponse")
(setf (get autofire::Lidar2EnemyResponse :definition-)
      "bool setStatus
---
bool received

")



(provide :autofire/Lidar2Enemy "5c20fa677eb90ce8ed6acc054aa17bb1")


