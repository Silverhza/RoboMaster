
(cl:in-package :asdf)

(defsystem "autofire-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ShootSwitchAction" :depends-on ("_package_ShootSwitchAction"))
    (:file "_package_ShootSwitchAction" :depends-on ("_package"))
    (:file "ShootSwitchActionFeedback" :depends-on ("_package_ShootSwitchActionFeedback"))
    (:file "_package_ShootSwitchActionFeedback" :depends-on ("_package"))
    (:file "ShootSwitchActionGoal" :depends-on ("_package_ShootSwitchActionGoal"))
    (:file "_package_ShootSwitchActionGoal" :depends-on ("_package"))
    (:file "ShootSwitchActionResult" :depends-on ("_package_ShootSwitchActionResult"))
    (:file "_package_ShootSwitchActionResult" :depends-on ("_package"))
    (:file "ShootSwitchFeedback" :depends-on ("_package_ShootSwitchFeedback"))
    (:file "_package_ShootSwitchFeedback" :depends-on ("_package"))
    (:file "ShootSwitchGoal" :depends-on ("_package_ShootSwitchGoal"))
    (:file "_package_ShootSwitchGoal" :depends-on ("_package"))
    (:file "ShootSwitchResult" :depends-on ("_package_ShootSwitchResult"))
    (:file "_package_ShootSwitchResult" :depends-on ("_package"))
    (:file "enemy_id" :depends-on ("_package_enemy_id"))
    (:file "_package_enemy_id" :depends-on ("_package"))
  ))