
(cl:in-package :asdf)

(defsystem "localize-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LaneMeasure" :depends-on ("_package_LaneMeasure"))
    (:file "_package_LaneMeasure" :depends-on ("_package"))
  ))