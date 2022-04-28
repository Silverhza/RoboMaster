
(cl:in-package :asdf)

(defsystem "autofire-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Lidar2Enemy" :depends-on ("_package_Lidar2Enemy"))
    (:file "_package_Lidar2Enemy" :depends-on ("_package"))
    (:file "ShootSwitch" :depends-on ("_package_ShootSwitch"))
    (:file "_package_ShootSwitch" :depends-on ("_package"))
  ))