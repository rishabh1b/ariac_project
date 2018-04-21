; Auto-generated. Do not edit!


(cl:in-package localisation-srv)


;//! \htmlinclude request_logical_pose-request.msg.html

(cl:defclass <request_logical_pose-request> (roslisp-msg-protocol:ros-message)
  ((request_msg
    :reader request_msg
    :initarg :request_msg
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass request_logical_pose-request (<request_logical_pose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <request_logical_pose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'request_logical_pose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localisation-srv:<request_logical_pose-request> is deprecated: use localisation-srv:request_logical_pose-request instead.")))

(cl:ensure-generic-function 'request_msg-val :lambda-list '(m))
(cl:defmethod request_msg-val ((m <request_logical_pose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:request_msg-val is deprecated.  Use localisation-srv:request_msg instead.")
  (request_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <request_logical_pose-request>) ostream)
  "Serializes a message object of type '<request_logical_pose-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request_msg) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <request_logical_pose-request>) istream)
  "Deserializes a message object of type '<request_logical_pose-request>"
    (cl:setf (cl:slot-value msg 'request_msg) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<request_logical_pose-request>)))
  "Returns string type for a service object of type '<request_logical_pose-request>"
  "localisation/request_logical_poseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'request_logical_pose-request)))
  "Returns string type for a service object of type 'request_logical_pose-request"
  "localisation/request_logical_poseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<request_logical_pose-request>)))
  "Returns md5sum for a message object of type '<request_logical_pose-request>"
  "c674f42d46687f11495636db257cd612")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'request_logical_pose-request)))
  "Returns md5sum for a message object of type 'request_logical_pose-request"
  "c674f42d46687f11495636db257cd612")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<request_logical_pose-request>)))
  "Returns full string definition for message of type '<request_logical_pose-request>"
  (cl:format cl:nil "bool request_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'request_logical_pose-request)))
  "Returns full string definition for message of type 'request_logical_pose-request"
  (cl:format cl:nil "bool request_msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <request_logical_pose-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <request_logical_pose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'request_logical_pose-request
    (cl:cons ':request_msg (request_msg msg))
))
;//! \htmlinclude request_logical_pose-response.msg.html

(cl:defclass <request_logical_pose-response> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (tgtorientation
    :reader tgtorientation
    :initarg :tgtorientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (tgtposition
    :reader tgtposition
    :initarg :tgtposition
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (order_completed
    :reader order_completed
    :initarg :order_completed
    :type cl:boolean
    :initform cl:nil)
   (conveyorPart
    :reader conveyorPart
    :initarg :conveyorPart
    :type cl:boolean
    :initform cl:nil)
   (noPartFound
    :reader noPartFound
    :initarg :noPartFound
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass request_logical_pose-response (<request_logical_pose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <request_logical_pose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'request_logical_pose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name localisation-srv:<request_logical_pose-response> is deprecated: use localisation-srv:request_logical_pose-response instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:position-val is deprecated.  Use localisation-srv:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:orientation-val is deprecated.  Use localisation-srv:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'tgtorientation-val :lambda-list '(m))
(cl:defmethod tgtorientation-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:tgtorientation-val is deprecated.  Use localisation-srv:tgtorientation instead.")
  (tgtorientation m))

(cl:ensure-generic-function 'tgtposition-val :lambda-list '(m))
(cl:defmethod tgtposition-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:tgtposition-val is deprecated.  Use localisation-srv:tgtposition instead.")
  (tgtposition m))

(cl:ensure-generic-function 'order_completed-val :lambda-list '(m))
(cl:defmethod order_completed-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:order_completed-val is deprecated.  Use localisation-srv:order_completed instead.")
  (order_completed m))

(cl:ensure-generic-function 'conveyorPart-val :lambda-list '(m))
(cl:defmethod conveyorPart-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:conveyorPart-val is deprecated.  Use localisation-srv:conveyorPart instead.")
  (conveyorPart m))

(cl:ensure-generic-function 'noPartFound-val :lambda-list '(m))
(cl:defmethod noPartFound-val ((m <request_logical_pose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader localisation-srv:noPartFound-val is deprecated.  Use localisation-srv:noPartFound instead.")
  (noPartFound m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <request_logical_pose-response>) ostream)
  "Serializes a message object of type '<request_logical_pose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tgtorientation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tgtposition) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'order_completed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'conveyorPart) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'noPartFound) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <request_logical_pose-response>) istream)
  "Deserializes a message object of type '<request_logical_pose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tgtorientation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tgtposition) istream)
    (cl:setf (cl:slot-value msg 'order_completed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'conveyorPart) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'noPartFound) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<request_logical_pose-response>)))
  "Returns string type for a service object of type '<request_logical_pose-response>"
  "localisation/request_logical_poseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'request_logical_pose-response)))
  "Returns string type for a service object of type 'request_logical_pose-response"
  "localisation/request_logical_poseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<request_logical_pose-response>)))
  "Returns md5sum for a message object of type '<request_logical_pose-response>"
  "c674f42d46687f11495636db257cd612")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'request_logical_pose-response)))
  "Returns md5sum for a message object of type 'request_logical_pose-response"
  "c674f42d46687f11495636db257cd612")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<request_logical_pose-response>)))
  "Returns full string definition for message of type '<request_logical_pose-response>"
  (cl:format cl:nil "geometry_msgs/Vector3 position~%geometry_msgs/Quaternion orientation~%geometry_msgs/Quaternion tgtorientation~%geometry_msgs/Vector3 tgtposition~%bool order_completed~%bool conveyorPart~%bool noPartFound~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'request_logical_pose-response)))
  "Returns full string definition for message of type 'request_logical_pose-response"
  (cl:format cl:nil "geometry_msgs/Vector3 position~%geometry_msgs/Quaternion orientation~%geometry_msgs/Quaternion tgtorientation~%geometry_msgs/Vector3 tgtposition~%bool order_completed~%bool conveyorPart~%bool noPartFound~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <request_logical_pose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tgtorientation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tgtposition))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <request_logical_pose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'request_logical_pose-response
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':tgtorientation (tgtorientation msg))
    (cl:cons ':tgtposition (tgtposition msg))
    (cl:cons ':order_completed (order_completed msg))
    (cl:cons ':conveyorPart (conveyorPart msg))
    (cl:cons ':noPartFound (noPartFound msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'request_logical_pose)))
  'request_logical_pose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'request_logical_pose)))
  'request_logical_pose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'request_logical_pose)))
  "Returns string type for a service object of type '<request_logical_pose>"
  "localisation/request_logical_pose")