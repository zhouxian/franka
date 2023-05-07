
(cl:in-package :asdf)

(defsystem "franka_interface_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :franka_interface_msgs-msg
)
  :components ((:file "_package")
    (:file "GetCurrentFrankaInterfaceStatusCmd" :depends-on ("_package_GetCurrentFrankaInterfaceStatusCmd"))
    (:file "_package_GetCurrentFrankaInterfaceStatusCmd" :depends-on ("_package"))
    (:file "GetCurrentRobotStateCmd" :depends-on ("_package_GetCurrentRobotStateCmd"))
    (:file "_package_GetCurrentRobotStateCmd" :depends-on ("_package"))
  ))