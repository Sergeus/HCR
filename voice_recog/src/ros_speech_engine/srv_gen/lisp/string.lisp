; Auto-generated. Do not edit!


(cl:in-package ros_speech_engine-srv)


;//! \htmlinclude string-request.msg.html

(cl:defclass <string-request> (roslisp-msg-protocol:ros-message)
  ((msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform ""))
)

(cl:defclass string-request (<string-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <string-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'string-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_speech_engine-srv:<string-request> is deprecated: use ros_speech_engine-srv:string-request instead.")))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <string-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_speech_engine-srv:msg-val is deprecated.  Use ros_speech_engine-srv:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <string-request>) ostream)
  "Serializes a message object of type '<string-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <string-request>) istream)
  "Deserializes a message object of type '<string-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<string-request>)))
  "Returns string type for a service object of type '<string-request>"
  "ros_speech_engine/stringRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'string-request)))
  "Returns string type for a service object of type 'string-request"
  "ros_speech_engine/stringRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<string-request>)))
  "Returns md5sum for a message object of type '<string-request>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'string-request)))
  "Returns md5sum for a message object of type 'string-request"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<string-request>)))
  "Returns full string definition for message of type '<string-request>"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'string-request)))
  "Returns full string definition for message of type 'string-request"
  (cl:format cl:nil "string msg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <string-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <string-request>))
  "Converts a ROS message object to a list"
  (cl:list 'string-request
    (cl:cons ':msg (msg msg))
))
;//! \htmlinclude string-response.msg.html

(cl:defclass <string-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass string-response (<string-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <string-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'string-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_speech_engine-srv:<string-response> is deprecated: use ros_speech_engine-srv:string-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <string-response>) ostream)
  "Serializes a message object of type '<string-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <string-response>) istream)
  "Deserializes a message object of type '<string-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<string-response>)))
  "Returns string type for a service object of type '<string-response>"
  "ros_speech_engine/stringResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'string-response)))
  "Returns string type for a service object of type 'string-response"
  "ros_speech_engine/stringResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<string-response>)))
  "Returns md5sum for a message object of type '<string-response>"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'string-response)))
  "Returns md5sum for a message object of type 'string-response"
  "7d96ed730776804754140b85e64c862e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<string-response>)))
  "Returns full string definition for message of type '<string-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'string-response)))
  "Returns full string definition for message of type 'string-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <string-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <string-response>))
  "Converts a ROS message object to a list"
  (cl:list 'string-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'string)))
  'string-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'string)))
  'string-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'string)))
  "Returns string type for a service object of type '<string>"
  "ros_speech_engine/string")