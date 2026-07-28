/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Robotics Laboratory.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// ROS 2 EusLisp C++ binding layer
// Based on the ROS 1 roseus.cpp by Kei Okada
// Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <memory>
#include <chrono>
#include <functional>

// ROS 2 headers - MUST be included BEFORE eus.h keyword masking
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialization.hpp>
#include <rcl/service.h>
#include <rcl_action/rcl_action.h>
#include <rmw/rmw.h>
#include <rcutils/logging.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <roseus/version.h>

// EusLisp keyword masking
#define class eus_class
#define throw eus_throw
#define export eus_export
#define vector eus_vector
#define string eus_string

#include "eus.h"
extern "C" {
pointer ___roseus(register context* ctx, int n, pointer* argv, pointer env);
void register_roseus() {
  char modname[] = "___roseus";
  return add_module_initializer(modname, (pointer(*)())___roseus);
}
byte* get_string(register pointer s) {
  if (isstring(s)) return (s->c.str.chars);
  if (issymbol(s)) return (s->c.sym.pname->c.str.chars);
  else error(E_NOSTRING);
  return NULL;
}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

using namespace std;

/***********************************************************
 *   Global state
 ************************************************************/

// Per-action-client data: holds rcl client, type supports, and libraries
struct ActionClientData {
  rcl_action_client_t client;
  const rosidl_action_type_support_t* action_ts;
  // Introspection for C struct allocation/init/fini
  const rosidl_typesupport_introspection_cpp::ServiceMembers* goal_svc_intro;
  const rosidl_typesupport_introspection_cpp::ServiceMembers* result_svc_intro;
  const rosidl_typesupport_introspection_cpp::ServiceMembers* cancel_svc_intro;
  const rosidl_typesupport_introspection_cpp::MessageMembers* feedback_msg_intro;
  const rosidl_typesupport_introspection_cpp::MessageMembers* status_msg_intro;
  // Keep shared libraries alive
  vector<shared_ptr<rcpputils::SharedLibrary>> libs;
};

// Per-action-server data
struct ActionServerData {
  rcl_action_server_t server;
  const rosidl_action_type_support_t* action_ts;
  const rosidl_typesupport_introspection_cpp::ServiceMembers* goal_svc_intro;
  const rosidl_typesupport_introspection_cpp::ServiceMembers* result_svc_intro;
  const rosidl_typesupport_introspection_cpp::ServiceMembers* cancel_svc_intro;
  const rosidl_typesupport_introspection_cpp::MessageMembers* feedback_msg_intro;
  const rosidl_typesupport_introspection_cpp::MessageMembers* status_msg_intro;
  vector<shared_ptr<rcpputils::SharedLibrary>> libs;
  // Goal handle management
  vector<rcl_action_goal_handle_t*> goal_handles;
  // Request headers for pending service requests
  rmw_request_id_t last_goal_request_header;
  rmw_request_id_t last_result_request_header;
  rmw_request_id_t last_cancel_request_header;
};

struct RoseusStaticData {
  RoseusStaticData() {}
  ~RoseusStaticData() {}
  shared_ptr<rclcpp::Node> node;
  shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  shared_ptr<rclcpp::Rate> rate;
  map<string, shared_ptr<rclcpp::GenericPublisher>> mapAdvertised;
  map<string, shared_ptr<rclcpp::GenericSubscription>> mapSubscribed;
  map<string, shared_ptr<rclcpp::ServiceBase>> mapServiced;
  map<string, shared_ptr<rclcpp::GenericClient>> mapServiceClients;
  map<string, shared_ptr<rclcpp::TimerBase>> mapTimered;
  map<string, shared_ptr<rclcpp::CallbackGroup>> mapCallbackGroup;
  map<string, shared_ptr<ActionClientData>> mapActionClients;
  map<string, shared_ptr<ActionServerData>> mapActionServers;
};

static RoseusStaticData s_staticdata;
static bool s_bInstalled = false;
#define s_node s_staticdata.node
#define s_executor s_staticdata.executor
#define s_rate s_staticdata.rate
#define s_mapAdvertised s_staticdata.mapAdvertised
#define s_mapSubscribed s_staticdata.mapSubscribed
#define s_mapServiced s_staticdata.mapServiced
#define s_mapServiceClients s_staticdata.mapServiceClients
#define s_mapTimered s_staticdata.mapTimered
#define s_mapCallbackGroup s_staticdata.mapCallbackGroup
#define s_mapActionClients s_staticdata.mapActionClients
#define s_mapActionServers s_staticdata.mapActionServers

pointer K_ROSEUS_MD5SUM, K_ROSEUS_DATATYPE, K_ROSEUS_DEFINITION;
pointer K_ROSEUS_SERIALIZATION_LENGTH, K_ROSEUS_SERIALIZE, K_ROSEUS_DESERIALIZE;
pointer K_ROSEUS_SERIALIZATION_LENGTH_CDR, K_ROSEUS_SERIALIZE_CDR, K_ROSEUS_DESERIALIZE_CDR;
pointer K_ROSEUS_INIT, K_ROSEUS_GET, K_ROSEUS_REQUEST, K_ROSEUS_RESPONSE;
pointer K_ROSEUS_GROUPNAME, K_ROSEUS_ONESHOT;
pointer K_ROSEUS_LAST_EXPECTED, K_ROSEUS_LAST_REAL;
pointer K_ROSEUS_CURRENT_EXPECTED, K_ROSEUS_CURRENT_REAL;
pointer K_ROSEUS_LAST_DURATION, K_ROSEUS_SEC, K_ROSEUS_NSEC;
pointer QANON, QNOOUT, QREPOVERSION, QROSDEBUG, QROSINFO, QROSWARN, QROSERROR, QROSFATAL;
extern pointer LAMCLOSURE;

#define isInstalledCheck \
  if (!s_bInstalled) { error(E_USER, "You must call (ros::roseus \"name\") before using ROS functions"); }

/***********************************************************
 *   Helper functions
 ************************************************************/

static string getString(pointer message, pointer method) {
  context* ctx = current_ctx;
  pointer r, curclass;
  if ((pointer)findmethod(ctx, method, classof(message), &curclass) != NIL) {
    r = csend(ctx, message, method, 0);
  } else if ((pointer)findmethod(ctx, K_ROSEUS_GET, classof(message), &curclass) != NIL) {
    r = csend(ctx, message, K_ROSEUS_GET, 1, method);
  } else {
    r = NULL;
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "could not find method for pointer %p", (void*)message);
  }
  if (!isstring(r)) {
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "method returned non-string");
  }
  string ret = (char*)get_string(r);
  return ret;
}

static int getInteger(pointer message, pointer method) {
  context* ctx = current_ctx;
  pointer a, curclass;
  vpush(message);
  a = (pointer)findmethod(ctx, method, classof(message), &curclass);
  if (a != NIL) {
    pointer r = csend(ctx, message, method, 0);
    vpop();
    return (ckintval(r));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "could not find method for pointer %p", (void*)message);
    vpop();
  }
  return 0;
}

/* Resolve a topic/service name using the node's namespace */
static string resolveName(const string& name) {
  if (!s_node) return name;
  // If the name already starts with /, it's absolute
  if (!name.empty() && name[0] == '/') return name;
  // Otherwise prefix with the node namespace
  string ns = s_node->get_namespace();
  if (ns == "/") return "/" + name;
  return ns + "/" + name;
}

/***********************************************************
 *   CDR Serialization helpers
 ************************************************************/

/* Serialize an EusLisp message to CDR via :serialize-cdr */
static rclcpp::SerializedMessage serializeEusMessage(pointer message) {
  context* ctx = current_ctx;
  pointer a, curclass;
  vpush(message);

  // Call :serialization-length-cdr to get the size
  a = (pointer)findmethod(ctx, K_ROSEUS_SERIALIZATION_LENGTH_CDR, classof(message), &curclass);
  if (a == NIL) {
    vpop();
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "Message does not have :serialization-length-cdr method");
    return rclcpp::SerializedMessage(0);
  }
  int len = getInteger(message, K_ROSEUS_SERIALIZATION_LENGTH_CDR);

  // Call :serialize-cdr to get the CDR bytes
  a = (pointer)findmethod(ctx, K_ROSEUS_SERIALIZE_CDR, classof(message), &curclass);
  if (a == NIL) {
    vpop();
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "Message does not have :serialize-cdr method");
    return rclcpp::SerializedMessage(0);
  }
  pointer r = csend(ctx, message, K_ROSEUS_SERIALIZE_CDR, 0);
  if (!isstring(r)) {
    vpop();
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), ":serialize-cdr returned non-string");
    return rclcpp::SerializedMessage(0);
  }

  uint8_t* data = (uint8_t*)get_string(r);
  size_t data_len = strlength(r);

  rclcpp::SerializedMessage serialized_msg(data_len);
  serialized_msg.get_rcl_serialized_message().buffer_length = data_len;
  memcpy(serialized_msg.get_rcl_serialized_message().buffer, data, data_len);

  vpop();
  return serialized_msg;
}

/* Deserialize CDR bytes into an EusLisp message instance via :deserialize-cdr */
static void deserializeEusMessage(pointer message, const std::shared_ptr<const rclcpp::SerializedMessage>& serialized_msg) {
  context* ctx = current_ctx;
  vpush(message);

  const uint8_t* data = serialized_msg->get_rcl_serialized_message().buffer;
  size_t data_len = serialized_msg->get_rcl_serialized_message().buffer_length;

  if (data_len == 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("roseus"), "empty message!");
    vpop();
    return;
  }

  pointer a, curclass;
  a = (pointer)findmethod(ctx, K_ROSEUS_DESERIALIZE_CDR, classof(message), &curclass);
  if (a == NIL) {
    vpop();
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "Message does not have :deserialize-cdr method");
    return;
  }

  pointer p = makestring((char*)data, data_len);
  vpush(p);
  pointer r = csend(ctx, message, K_ROSEUS_DESERIALIZE_CDR, 1, p);
  vpop();  // p
  if (r == NIL) {
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), ":deserialize-cdr returned nil");
  }
  vpop();  // message
}

/***********************************************************
 *   Callback function resolution
 ************************************************************/

/* Extract callable from EusLisp callback specification */
static pointer resolveCallback(context* ctx, pointer scb) {
  if (piscode(scb)) {
    return scb;
  } else if (ccar(scb) == LAMCLOSURE) {
    if (ccar(ccdr(scb)) != NIL) {
      return ccar(ccdr(scb));  // named function
    } else {
      return scb;  // lambda
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "callback function install error");
    return NIL;
  }
}

/* Prevent GC of callback function and its arguments */
static void protectCallback(context* ctx, pointer scb, pointer args) {
  pointer p = gensym(ctx);
  setval(ctx, intern(ctx, (char*)(p->c.sym.pname->c.str.chars), strlen((char*)(p->c.sym.pname->c.str.chars)), lisppkg),
         cons(ctx, scb, args));
}

/***********************************************************
 *   Signal handler
 ************************************************************/

static void roseusSignalHandler(int sig) {
  context* ctx = euscontexts[thr_self()];
  ctx->intsig = sig;
}

/***********************************************************
 *   Core EUSLISP functions
 ************************************************************/

pointer ROSEUS(register context* ctx, int n, pointer* argv) {
  char name[256] = "";
  uint32_t options = 0;
  int cargc = 0;
  char* cargv[32];

  if (s_bInstalled) {
    RCLCPP_WARN(s_node->get_logger(), "ROSEUS is already installed as %s",
                s_node->get_fully_qualified_name());
    return (T);
  }

  ckarg(3);
  if (isstring(argv[0]))
    strncpy(name, (char*)(argv[0]->c.str.chars), 255);
  else error(E_NOSTRING);
  options = ckintval(argv[1]);
  pointer p = argv[2];
  if (islist(p)) {
    while (1) {
      if (!iscons(p)) break;
      cargv[cargc] = (char*)((ccar(p))->c.str.chars);
      cargc++;
      p = ccdr(p);
    }
  } else error(E_NOSEQ);

  // Convert invalid node name characters to _
  for (unsigned int i = 0; i < strlen(name); i++)
    if (!(isalpha(name[i]) || isdigit(name[i]))) name[i] = '_';

  // Define keywords for EusLisp method dispatch
  K_ROSEUS_MD5SUM = defkeyword(ctx, "MD5SUM-");
  K_ROSEUS_DATATYPE = defkeyword(ctx, "DATATYPE-");
  K_ROSEUS_DEFINITION = defkeyword(ctx, "DEFINITION-");
  K_ROSEUS_SERIALIZATION_LENGTH = defkeyword(ctx, "SERIALIZATION-LENGTH");
  K_ROSEUS_SERIALIZE = defkeyword(ctx, "SERIALIZE");
  K_ROSEUS_DESERIALIZE = defkeyword(ctx, "DESERIALIZE");
  K_ROSEUS_SERIALIZATION_LENGTH_CDR = defkeyword(ctx, "SERIALIZATION-LENGTH-CDR");
  K_ROSEUS_SERIALIZE_CDR = defkeyword(ctx, "SERIALIZE-CDR");
  K_ROSEUS_DESERIALIZE_CDR = defkeyword(ctx, "DESERIALIZE-CDR");
  K_ROSEUS_GET = defkeyword(ctx, "GET");
  K_ROSEUS_INIT = defkeyword(ctx, "INIT");
  K_ROSEUS_REQUEST = defkeyword(ctx, "REQUEST");
  K_ROSEUS_RESPONSE = defkeyword(ctx, "RESPONSE");
  K_ROSEUS_GROUPNAME = defkeyword(ctx, "GROUPNAME");
  K_ROSEUS_ONESHOT = defkeyword(ctx, "ONESHOT");
  K_ROSEUS_LAST_EXPECTED = defkeyword(ctx, "LAST-EXPECTED");
  K_ROSEUS_LAST_REAL = defkeyword(ctx, "LAST-REAL");
  K_ROSEUS_CURRENT_EXPECTED = defkeyword(ctx, "CURRENT-EXPECTED");
  K_ROSEUS_CURRENT_REAL = defkeyword(ctx, "CURRENT-REAL");
  K_ROSEUS_LAST_DURATION = defkeyword(ctx, "LAST-DURATION");
  K_ROSEUS_SEC = defkeyword(ctx, "SEC");
  K_ROSEUS_NSEC = defkeyword(ctx, "NSEC");

  // Clear all maps
  s_mapAdvertised.clear();
  s_mapSubscribed.clear();
  s_mapServiced.clear();
  s_mapServiceClients.clear();
  s_mapTimered.clear();
  s_mapCallbackGroup.clear();
  s_mapActionClients.clear();
  s_mapActionServers.clear();

  setlocale(LC_ALL, "");

  // Initialize rclcpp
  rclcpp::InitOptions init_options;
  if (!rclcpp::ok()) {
    rclcpp::init(cargc, cargv, init_options);
  }

  // Create node options
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  // Handle anonymous name
  string node_name(name);
  if (options & 1) {  // AnonymousName flag
    auto now = std::chrono::steady_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    node_name += "_" + std::to_string(ns);
  }

  try {
    s_node = rclcpp::Node::make_shared(node_name, node_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("roseus"), "%s", e.what());
    error(E_MISMATCHARG);
    return (NIL);
  }

  s_executor.reset(new rclcpp::executors::SingleThreadedExecutor());
  s_executor->add_node(s_node);
  s_rate.reset(new rclcpp::Rate(50));

  s_bInstalled = true;

  // Install signal handler for SIGINT
  signal(SIGINT, roseusSignalHandler);

  return (T);
}

pointer ROSEUS_SPIN(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  while (ctx->intsig == 0 && rclcpp::ok()) {
    s_executor->spin_some();
    s_rate->sleep();
  }
  return (NIL);
}

pointer ROSEUS_SPINONCE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg2(0, 1);

  if (n > 0) {
    string groupname;
    if (isstring(argv[0])) groupname.assign((char*)get_string(argv[0]));
    else error(E_NOSTRING);

    auto it = s_mapCallbackGroup.find(groupname);
    if (it == s_mapCallbackGroup.end()) {
      RCLCPP_ERROR(s_node->get_logger(), "Groupname %s is missing", groupname.c_str());
      return (T);
    }
    // spin_some processes callbacks for all groups that have pending work
    s_executor->spin_some();
    return (NIL);
  } else {
    s_executor->spin_some();
    return (NIL);
  }
}

pointer ROSEUS_TIME_NOW(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  pointer timevec;
  rclcpp::Time t = s_node->now();

  timevec = makevector(C_INTVECTOR, 2);
  vpush(timevec);
  int32_t sec = static_cast<int32_t>(t.seconds());
  uint32_t nsec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL);
  timevec->c.ivec.iv[0] = sec;
  timevec->c.ivec.iv[1] = nsec;
  vpop();
  return (timevec);
}

pointer ROSEUS_RATE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  numunion nu;
  ckarg(1);
  float freq = ckfltval(argv[0]);
  s_rate.reset(new rclcpp::Rate(freq));
  return (T);
}

pointer ROSEUS_SLEEP(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  s_rate->sleep();
  return (T);
}

pointer ROSEUS_DURATION_SLEEP(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  numunion nu;
  ckarg(1);
  float sleep_sec = ckfltval(argv[0]);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(sleep_sec)));
  return (T);
}

pointer ROSEUS_OK(register context* ctx, int n, pointer* argv) {
  if (rclcpp::ok()) {
    return (T);
  } else {
    return (NIL);
  }
}

/***********************************************************
 *   Logging
 ************************************************************/

#define def_rosconsole_formatter(funcname, loglevel)                          \
  pointer funcname(register context* ctx, int n, pointer* argv) {             \
    pointer *argv2, msg;                                                      \
    int argc2;                                                                \
    argc2 = n + 1;                                                            \
    argv2 = (pointer*)malloc(sizeof(pointer) * argc2);                        \
    argv2[0] = NIL;                                                           \
    for (int i = 0; i < n; i++) argv2[i + 1] = argv[i];                       \
    msg = XFORMAT(ctx, argc2, argv2);                                         \
    if (s_bInstalled) {                                                       \
      RCUTILS_LOG_COND_NAMED(loglevel, RCUTILS_LOG_CONDITION_EMPTY,           \
                             RCUTILS_LOG_CONDITION_EMPTY, s_node->get_name(), \
                             "%s", msg->c.str.chars);                         \
    } else {                                                                  \
      RCUTILS_LOG_COND_NAMED(loglevel, RCUTILS_LOG_CONDITION_EMPTY,           \
                             RCUTILS_LOG_CONDITION_EMPTY, "roseus",           \
                             "%s", msg->c.str.chars);                         \
    }                                                                         \
    free(argv2);                                                              \
    return (T);                                                               \
  }

def_rosconsole_formatter(ROSEUS_ROSDEBUG, RCUTILS_LOG_SEVERITY_DEBUG);
def_rosconsole_formatter(ROSEUS_ROSINFO, RCUTILS_LOG_SEVERITY_INFO);
def_rosconsole_formatter(ROSEUS_ROSWARN, RCUTILS_LOG_SEVERITY_WARN);
def_rosconsole_formatter(ROSEUS_ROSERROR, RCUTILS_LOG_SEVERITY_ERROR);
def_rosconsole_formatter(ROSEUS_ROSFATAL, RCUTILS_LOG_SEVERITY_FATAL);

pointer ROSEUS_SET_LOGGER_LEVEL(register context* ctx, int n, pointer* argv) {
  ckarg(2);
  string logger;
  if (isstring(argv[0])) logger.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  int log_level = intval(argv[1]);

  rcutils_ret_t ret;
  switch (log_level) {
    case 1:
      ret = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);
      break;
    case 2:
      ret = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_INFO);
      break;
    case 3:
      ret = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_WARN);
      break;
    case 4:
      ret = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_ERROR);
      break;
    case 5:
      ret = rcutils_logging_set_logger_level(logger.c_str(), RCUTILS_LOG_SEVERITY_FATAL);
      break;
    default:
      return (NIL);
  }

  return (ret == RCUTILS_RET_OK) ? T : NIL;
}

pointer ROSEUS_EXIT(register context* ctx, int n, pointer* argv) {
  if (s_bInstalled) {
    RCLCPP_INFO(s_node->get_logger(), "exiting roseus %ld", (n == 0) ? 0L : (long)ckintval(argv[0]));
    s_mapAdvertised.clear();
    s_mapSubscribed.clear();
    s_mapServiced.clear();
    s_mapServiceClients.clear();
    s_mapTimered.clear();
    s_mapCallbackGroup.clear();
    s_mapActionClients.clear();
    s_mapActionServers.clear();
    s_executor.reset();
    s_node.reset();
    rclcpp::shutdown();
    s_bInstalled = false;
  }
  if (n == 0) _exit(0);
  else _exit(ckintval(argv[0]));
}

/***********************************************************
 *   Publisher / Subscriber
 ************************************************************/

pointer ROSEUS_ADVERTISE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string topicname;
  pointer message;
  int queuesize = 1;
  bool latch = false;

  ckarg2(2, 4);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  message = argv[1];
  if (n > 2) {
    queuesize = ckintval(argv[2]);
  }
  if (n > 3) {
    latch = (argv[3] != NIL);
  }

  RCLCPP_DEBUG(s_node->get_logger(), "advertise %s %d %d", topicname.c_str(), queuesize, latch);
  if (s_mapAdvertised.find(topicname) != s_mapAdvertised.end()) {
    RCLCPP_WARN(s_node->get_logger(), "topic %s already advertised", topicname.c_str());
    return (NIL);
  }

  // Get the ROS 2 datatype string (e.g. "std_msgs/msg/String")
  string datatype = getString(message, K_ROSEUS_DATATYPE);

  // Build QoS
  rclcpp::QoS qos(queuesize);
  qos.reliable();
  if (latch) {
    qos.transient_local();
  }

  auto pub = s_node->create_generic_publisher(topicname, datatype, qos);
  if (pub) {
    s_mapAdvertised[topicname] = pub;
  } else {
    RCLCPP_ERROR(s_node->get_logger(), "failed to create publisher for %s", topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_UNADVERTISE(register context* ctx, int n, pointer* argv) {
  string topicname;
  ckarg(1);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = s_mapAdvertised.erase(topicname) > 0;
  return (bSuccess ? T : NIL);
}

pointer ROSEUS_PUBLISH(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string topicname;
  pointer emessage;

  ckarg(2);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  emessage = argv[1];

  bool bSuccess = false;
  auto it = s_mapAdvertised.find(topicname);
  if (it != s_mapAdvertised.end()) {
    auto publisher = it->second;
    rclcpp::SerializedMessage serialized_msg = serializeEusMessage(emessage);
    publisher->publish(serialized_msg);
    bSuccess = true;
  }

  if (!bSuccess) {
    RCLCPP_ERROR(s_node->get_logger(),
                 "attempted to publish to topic %s, which was not "
                 "previously advertised. call (ros::advertise \"%s\") first.",
                 topicname.c_str(), topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_SUBSCRIBE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string topicname;
  pointer message, fncallback, args;
  int queuesize = 1;
  rclcpp::CallbackGroup::SharedPtr cbg = nullptr;

  // arguments:
  // topicname message_type callbackfunc args0 ... argsN [ queuesize ] [ :groupname groupname ]
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  // Parse trailing keyword arguments
  if (n > 1 && issymbol(argv[n - 2]) && isstring(argv[n - 1])) {
    if (argv[n - 2] == K_ROSEUS_GROUPNAME) {
      string groupname;
      groupname.assign((char*)get_string(argv[n - 1]));
      auto it = s_mapCallbackGroup.find(groupname);
      if (it != s_mapCallbackGroup.end()) {
        RCLCPP_DEBUG(s_node->get_logger(), "subscribe with groupname=%s", groupname.c_str());
        cbg = it->second;
      } else {
        RCLCPP_ERROR(s_node->get_logger(),
                     "Groupname %s is missing. Topic %s is not subscribed. "
                     "Call (ros::create-nodehandle \"%s\") first.",
                     groupname.c_str(), topicname.c_str(), groupname.c_str());
        return (NIL);
      }
      n -= 2;
    }
  }
  if (isint(argv[n - 1])) {
    queuesize = ckintval(argv[n - 1]);
    n--;
  }
  RCLCPP_DEBUG(s_node->get_logger(), "subscribe %s queuesize=%d", topicname.c_str(), queuesize);

  message = argv[1];
  fncallback = argv[2];
  args = NIL;
  for (int i = n - 1; i >= 3; i--) args = cons(ctx, argv[i], args);

  // Get the ROS 2 datatype
  string datatype = getString(message, K_ROSEUS_DATATYPE);

  // Resolve the callback function
  pointer scb = resolveCallback(ctx, fncallback);
  protectCallback(ctx, fncallback, args);

  // Protect the message class from GC
  vpush(message);

  // Build QoS
  rclcpp::QoS qos(queuesize);
  qos.reliable();

  // Create subscription options with callback group
  rclcpp::SubscriptionOptions sub_options;
  if (cbg) {
    sub_options.callback_group = cbg;
  }

  // Create a generic subscription with CDR callback
  // We capture scb, args, and message (class template) for the callback
  auto sub = s_node->create_generic_subscription(
      topicname, datatype, qos,
      [scb, args, message](std::shared_ptr<const rclcpp::SerializedMessage> serialized_msg) {
        context* ctx = current_ctx;
        if (ctx != euscontexts[0]) {
          RCLCPP_WARN(rclcpp::get_logger("roseus"), "ctx is not correct %ld", (long)thr_self());
        }

        // Create new instance of the message class
        pointer msg_instance;
        vpush(message);
        if (isclass(message)) {
          msg_instance = makeobject(message);
          vpush(msg_instance);
          csend(ctx, msg_instance, K_ROSEUS_INIT, 0);
          vpop();  // msg_instance
        } else {
          RCLCPP_WARN(rclcpp::get_logger("roseus"), "message template must be class");
          msg_instance = message;
        }
        vpop();  // message

        // Deserialize CDR into the EusLisp message
        deserializeEusMessage(msg_instance, serialized_msg);

        // Call the EusLisp callback function
        pointer argp = args;
        int argc = 0;
        vpush(msg_instance);
        while (argp != NIL) {
          ckpush(ccar(argp));
          argp = ccdr(argp);
          argc++;
        }
        vpush(msg_instance);
        argc++;

        ufuncall(ctx, (ctx->callfp ? ctx->callfp->form : NIL),
                 scb, (pointer)(ctx->vsp - argc), NULL, argc);
        while (argc-- > 0) vpop();
        vpop();  // msg_instance
      },
      sub_options);

  vpop();  // message

  if (sub) {
    s_mapSubscribed[topicname] = sub;
  } else {
    RCLCPP_ERROR(s_node->get_logger(), "failed to subscribe to %s", topicname.c_str());
  }

  return (T);
}

pointer ROSEUS_UNSUBSCRIBE(register context* ctx, int n, pointer* argv) {
  string topicname;
  ckarg(1);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  bool bSuccess = s_mapSubscribed.erase(topicname) > 0;
  return (bSuccess ? T : NIL);
}

pointer ROSEUS_GETNUMPUBLISHERS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string topicname;
  ckarg(1);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  size_t count = s_node->count_publishers(topicname);
  return makeint(count);
}

pointer ROSEUS_GETNUMSUBSCRIBERS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string topicname;
  ckarg(1);
  if (isstring(argv[0])) topicname = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  size_t count = s_node->count_subscribers(topicname);
  return makeint(count);
}

/***********************************************************
 *   Service
 ************************************************************/

pointer ROSEUS_WAIT_FOR_SERVICE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string service;
  numunion nu;

  ckarg2(1, 2);
  if (isstring(argv[0])) service = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  float timeout = -1;
  if (n > 1 && argv[1] != NIL)
    timeout = ckfltval(argv[1]);

  // Use rcl_service_server_is_available to check availability
  // We need an rcl_client_t for this, but we can use the graph API instead
  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto names_and_types = s_node->get_service_names_and_types();
    for (const auto& entry : names_and_types) {
      if (entry.first == service) {
        return (T);
      }
    }

    if (timeout >= 0) {
      auto elapsed = std::chrono::steady_clock::now() - start;
      if (std::chrono::duration<double>(elapsed).count() >= timeout) {
        return (NIL);
      }
    }
    if (!rclcpp::ok()) return (NIL);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  return (NIL);
}

pointer ROSEUS_SERVICE_EXISTS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string service;
  ckarg(1);
  if (isstring(argv[0])) service = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  auto names_and_types = s_node->get_service_names_and_types();
  for (const auto& entry : names_and_types) {
    if (entry.first == service) {
      return (T);
    }
  }
  return (NIL);
}

pointer ROSEUS_SERVICE_CALL(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string service;
  pointer emessage;
  bool persist = false;

  ckarg2(2, 3);
  if (isstring(argv[0])) service = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  emessage = argv[1];
  if (n > 2) {
    persist = (argv[2] != NIL);
  }

  // Get service datatype from the request message
  // The request's datatype is "pkg/srv/SrvNameRequest", but we need "pkg/srv/SrvName"
  string datatype = getString(emessage, K_ROSEUS_DATATYPE);
  {
    const string suffix = "Request";
    if (datatype.size() > suffix.size() &&
        datatype.compare(datatype.size() - suffix.size(), suffix.size(), suffix) == 0) {
      datatype = datatype.substr(0, datatype.size() - suffix.size());
    }
  }

  vpush(emessage);

  // Create response instance
  pointer response = csend(ctx, emessage, K_ROSEUS_RESPONSE, 0);
  vpush(response);

  // Find or create service client
  shared_ptr<rclcpp::GenericClient> client;
  auto it = s_mapServiceClients.find(service);
  if (persist && it != s_mapServiceClients.end()) {
    client = std::dynamic_pointer_cast<rclcpp::GenericClient>(it->second);
  }

  if (!client) {
    client = s_node->create_generic_client(service, datatype);
    if (persist) {
      s_mapServiceClients[service] = client;
    }
  }

  // Wait for service
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(s_node->get_logger(), "service %s not available", service.c_str());
    vpop();  // response
    vpop();  // emessage
    return (NIL);
  }

  // Get type support for rmw_serialize/rmw_deserialize
  auto ts_lib = rclcpp::get_typesupport_library(datatype, "rosidl_typesupport_cpp");
  auto srv_ts = rclcpp::get_service_typesupport_handle(
      datatype, "rosidl_typesupport_cpp", *ts_lib);

  // Get introspection type support for C struct allocation
  auto intro_lib = rclcpp::get_typesupport_library(
      datatype, "rosidl_typesupport_introspection_cpp");
  auto intro_ts = rclcpp::get_service_typesupport_handle(
      datatype, "rosidl_typesupport_introspection_cpp", *intro_lib);
  auto svc_members = static_cast<
      const rosidl_typesupport_introspection_cpp::ServiceMembers*>(intro_ts->data);

  // Serialize EusLisp request to CDR
  rclcpp::SerializedMessage serialized_req = serializeEusMessage(emessage);

  // Allocate C struct for request and convert CDR → C struct
  auto req_members = svc_members->request_members_;
  std::vector<uint8_t> req_buf(req_members->size_of_, 0);
  req_members->init_function(
      req_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_ret_t rmw_rc = rmw_deserialize(
      &serialized_req.get_rcl_serialized_message(),
      srv_ts->request_typesupport,
      req_buf.data());

  if (rmw_rc != RMW_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(),
                 "failed to deserialize request for service %s", service.c_str());
    req_members->fini_function(req_buf.data());
    vpop();  // response
    vpop();  // emessage
    return (NIL);
  }

  // Send request (GenericClient expects a C struct pointer)
  auto future_and_id = client->async_send_request(req_buf.data());

  // Spin until we get a response
  auto ret_code = s_executor->spin_until_future_complete(
      future_and_id, std::chrono::seconds(30));

  if (ret_code != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(s_node->get_logger(),
                 "service call to %s failed or timed out", service.c_str());
    client->remove_pending_request(future_and_id.request_id);
    req_members->fini_function(req_buf.data());
    vpop();  // response
    vpop();  // emessage
    return (NIL);
  }

  // Get response (shared_ptr<void> pointing to C struct)
  auto raw_response = future_and_id.get();

  if (raw_response) {
    // Serialize C struct response to CDR via rmw_serialize
    rcl_serialized_message_t resp_serialized = rmw_get_zero_initialized_serialized_message();
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rmw_serialized_message_init(&resp_serialized, 256, &allocator);

    rmw_rc = rmw_serialize(
        raw_response.get(), srv_ts->response_typesupport, &resp_serialized);

    if (rmw_rc == RMW_RET_OK) {
      // Wrap in SerializedMessage for deserializeEusMessage
      auto serialized_resp = std::make_shared<rclcpp::SerializedMessage>(
          resp_serialized.buffer_length);
      auto& rcl_msg = serialized_resp->get_rcl_serialized_message();
      memcpy(rcl_msg.buffer, resp_serialized.buffer, resp_serialized.buffer_length);
      rcl_msg.buffer_length = resp_serialized.buffer_length;

      // CDR → EusLisp response
      deserializeEusMessage(response, serialized_resp);
    } else {
      RCLCPP_ERROR(s_node->get_logger(),
                   "failed to serialize response for service %s", service.c_str());
    }

    rmw_serialized_message_fini(&resp_serialized);
  }

  req_members->fini_function(req_buf.data());
  vpop();  // response
  vpop();  // emessage

  return (response);
}

pointer ROSEUS_ADVERTISE_SERVICE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string service;
  pointer emessage;
  pointer fncallback, args;
  rclcpp::CallbackGroup::SharedPtr cbg = nullptr;

  if (isstring(argv[0])) service = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  emessage = argv[1];
  fncallback = argv[2];

  // Parse trailing keyword arguments
  if (n >= 5 && issymbol(argv[n - 2]) && isstring(argv[n - 1])) {
    if (argv[n - 2] == K_ROSEUS_GROUPNAME) {
      string groupname;
      groupname.assign((char*)get_string(argv[n - 1]));
      auto it = s_mapCallbackGroup.find(groupname);
      if (it != s_mapCallbackGroup.end()) {
        RCLCPP_DEBUG(s_node->get_logger(), "advertising service with groupname=%s",
                     groupname.c_str());
        cbg = it->second;
      } else {
        RCLCPP_ERROR(s_node->get_logger(),
                     "Groupname \"%s\" is missing. Service %s is not advertised. "
                     "Call (ros::create-nodehandle \"%s\") first.",
                     groupname.c_str(), service.c_str(), groupname.c_str());
        return (NIL);
      }
      n -= 2;
    }
  }

  args = NIL;
  for (int i = n - 1; i >= 3; i--) args = cons(ctx, argv[i], args);

  if (s_mapServiced.find(service) != s_mapServiced.end()) {
    RCLCPP_INFO(s_node->get_logger(), "service %s already advertised", service.c_str());
    return (NIL);
  }

  // Get service datatype (e.g. "pkg/srv/SrvName")
  string datatype = getString(emessage, K_ROSEUS_DATATYPE);

  // Get request and response class
  vpush(emessage);
  pointer request_class = csend(ctx, emessage, K_ROSEUS_GET, 1, K_ROSEUS_REQUEST);
  pointer response_class = csend(ctx, emessage, K_ROSEUS_GET, 1, K_ROSEUS_RESPONSE);
  vpop();  // emessage

  pointer scb = resolveCallback(ctx, fncallback);
  protectCallback(ctx, fncallback, args);

  // Protect classes from GC
  vpush(request_class);
  vpush(response_class);

  // Get type support for rmw_serialize/rmw_deserialize
  auto ts_lib = rclcpp::get_typesupport_library(datatype, "rosidl_typesupport_cpp");
  auto srv_ts = rclcpp::get_service_typesupport_handle(
      datatype, "rosidl_typesupport_cpp", *ts_lib);

  // Get introspection type support for C struct allocation
  auto intro_lib = rclcpp::get_typesupport_library(
      datatype, "rosidl_typesupport_introspection_cpp");
  auto intro_ts = rclcpp::get_service_typesupport_handle(
      datatype, "rosidl_typesupport_introspection_cpp", *intro_lib);
  auto svc_members = static_cast<
      const rosidl_typesupport_introspection_cpp::ServiceMembers*>(intro_ts->data);

  // Use rcl layer for generic service handling
  auto srv_node = s_node->get_node_base_interface();

  rcl_service_t* rcl_srv = new rcl_service_t();
  *rcl_srv = rcl_get_zero_initialized_service();

  rcl_service_options_t srv_options = rcl_service_get_default_options();

  rcl_ret_t rc = rcl_service_init(
      rcl_srv,
      srv_node->get_rcl_node_handle(),
      srv_ts,
      service.c_str(),
      &srv_options);

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to create service %s: %s",
                 service.c_str(), rcl_get_error_string().str);
    rcl_reset_error();
    delete rcl_srv;
    vpop();  // response_class
    vpop();  // request_class
    return (NIL);
  }

  auto srv_shared = std::shared_ptr<rcl_service_t>(rcl_srv,
                                                   [srv_node](rcl_service_t* srv) {
                                                     (void)rcl_service_fini(srv, srv_node->get_rcl_node_handle());
                                                     delete srv;
                                                   });

  // Poll for incoming requests via a wall timer
  auto timer = s_node->create_wall_timer(
      std::chrono::milliseconds(1),
      [srv_shared, ts_lib, intro_lib, srv_ts, svc_members,
       scb, args, request_class, response_class, service]() {
        context* ctx = current_ctx;
        auto req_members = svc_members->request_members_;
        auto resp_members = svc_members->response_members_;

        // Allocate C struct for request
        std::vector<uint8_t> req_buf(req_members->size_of_, 0);
        req_members->init_function(
            req_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

        rmw_service_info_t request_header;
        memset(&request_header, 0, sizeof(request_header));

        rcl_ret_t rc = rcl_take_request_with_info(
            srv_shared.get(), &request_header, req_buf.data());

        if (rc == RCL_RET_OK) {
          // C struct request → CDR
          rcl_serialized_message_t serialized_req_buf =
              rmw_get_zero_initialized_serialized_message();
          rcutils_allocator_t allocator = rcutils_get_default_allocator();
          rmw_serialized_message_init(&serialized_req_buf, 256, &allocator);

          rmw_ret_t rmw_rc = rmw_serialize(
              req_buf.data(), srv_ts->request_typesupport, &serialized_req_buf);

          if (rmw_rc != RMW_RET_OK) {
            RCLCPP_ERROR(rclcpp::get_logger("roseus"),
                         "service %s: failed to serialize request to CDR", service.c_str());
            rmw_serialized_message_fini(&serialized_req_buf);
            req_members->fini_function(req_buf.data());
            return;
          }

          // Create EusLisp request instance and deserialize CDR into it
          pointer req_instance = makeobject(
              isclass(request_class) ? request_class : classof(request_class));
          vpush(req_instance);
          csend(ctx, req_instance, K_ROSEUS_INIT, 0);

          auto sm = std::make_shared<rclcpp::SerializedMessage>(
              serialized_req_buf.buffer_length);
          auto& rcl_msg = sm->get_rcl_serialized_message();
          memcpy(rcl_msg.buffer, serialized_req_buf.buffer,
                 serialized_req_buf.buffer_length);
          rcl_msg.buffer_length = serialized_req_buf.buffer_length;
          rmw_serialized_message_fini(&serialized_req_buf);

          deserializeEusMessage(req_instance, sm);

          // Call EusLisp callback: (funcall callback request ...args)
          // Callback should return response instance
          if (!(issymbol(scb) || piscode(scb) || ccar(scb) == LAMCLOSURE)) {
            RCLCPP_ERROR(rclcpp::get_logger("roseus"),
                         "can't find service callback function");
            vpop();  // req_instance
            req_members->fini_function(req_buf.data());
            return;
          }

          pointer argp = args;
          int argc = 0;
          while (argp != NIL) {
            ckpush(ccar(argp));
            argp = ccdr(argp);
            argc++;
          }
          ckpush(req_instance);
          argc++;

          pointer eus_response = ufuncall(
              ctx, (ctx->callfp ? ctx->callfp->form : NIL),
              scb, (pointer)(ctx->vsp - argc), NULL, argc);
          while (argc-- > 0) vpop();
          vpush(eus_response);

          // Check if callback returned a valid response (must be a message with :serialize-cdr)
          pointer resp_curclass;
          if (eus_response == NIL || !ispointer(eus_response) ||
              (pointer)findmethod(ctx, K_ROSEUS_SERIALIZE_CDR,
                                  classof(eus_response), &resp_curclass) == NIL) {
            RCLCPP_WARN(rclcpp::get_logger("roseus"),
                        "service %s: callback did not return a valid response object, "
                        "sending default response",
                        service.c_str());
            // Send default (zero-initialized) response
            std::vector<uint8_t> resp_buf(resp_members->size_of_, 0);
            resp_members->init_function(
                resp_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
            rcl_send_response(
                srv_shared.get(), &request_header.request_id, resp_buf.data());
            resp_members->fini_function(resp_buf.data());
            vpop();  // eus_response
            vpop();  // req_instance
            req_members->fini_function(req_buf.data());
            return;
          }

          // Serialize EusLisp response to CDR
          rclcpp::SerializedMessage resp_cdr = serializeEusMessage(eus_response);

          // CDR → C struct for response
          std::vector<uint8_t> resp_buf(resp_members->size_of_, 0);
          resp_members->init_function(
              resp_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

          rmw_rc = rmw_deserialize(
              &resp_cdr.get_rcl_serialized_message(),
              srv_ts->response_typesupport,
              resp_buf.data());

          if (rmw_rc == RMW_RET_OK) {
            rcl_ret_t send_rc = rcl_send_response(
                srv_shared.get(), &request_header.request_id, resp_buf.data());
            if (send_rc != RCL_RET_OK) {
              RCLCPP_ERROR(rclcpp::get_logger("roseus"),
                           "service %s: failed to send response: %s",
                           service.c_str(), rcl_get_error_string().str);
              rcl_reset_error();
            }
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("roseus"),
                         "service %s: failed to deserialize response CDR",
                         service.c_str());
          }

          resp_members->fini_function(resp_buf.data());
          vpop();  // eus_response
          vpop();  // req_instance
        }

        req_members->fini_function(req_buf.data());
      });

  s_mapTimered["__srv__" + service] = timer;

  vpop();  // response_class
  vpop();  // request_class

  return (T);
}

pointer ROSEUS_UNADVERTISE_SERVICE(register context* ctx, int n, pointer* argv) {
  string service;
  ckarg(1);
  if (isstring(argv[0])) service = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  RCLCPP_DEBUG(rclcpp::get_logger("roseus"), "unadvertise-service %s", service.c_str());

  // Remove the polling timer
  s_mapTimered.erase("__srv__" + service);
  bool bSuccess = s_mapServiced.erase(service) > 0;

  return (bSuccess ? T : NIL);
}

/***********************************************************
 *   Parameters
 ************************************************************/

pointer ROSEUS_SET_PARAM(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string key;
  numunion nu;

  ckarg(2);
  if (isstring(argv[0])) key.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  // Declare parameter if not already declared
  if (!s_node->has_parameter(key)) {
    // Determine type from EusLisp value
    if (isstring(argv[1])) {
      s_node->declare_parameter<string>(key, string((char*)get_string(argv[1])));
    } else if (isint(argv[1])) {
      s_node->declare_parameter<int64_t>(key, (int64_t)intval(argv[1]));
    } else if (isflt(argv[1])) {
      s_node->declare_parameter<double>(key, (double)fltval(argv[1]));
    } else if (argv[1] == T) {
      s_node->declare_parameter<bool>(key, true);
    } else if (argv[1] == NIL) {
      s_node->declare_parameter<bool>(key, false);
    } else if (islist(argv[1])) {
      // Try to convert list to vector of doubles or strings
      // For simplicity, convert to string representation
      pointer dest = (pointer)mkstream(ctx, K_OUT, makebuffer(256));
      vpush(dest);
      prinx(ctx, argv[1], dest);
      pointer str = makestring((char*)dest->c.stream.buffer->c.str.chars,
                               intval(dest->c.stream.count));
      vpop();  // dest
      s_node->declare_parameter<string>(key, string((char*)get_string(str)));
    } else {
      s_node->declare_parameter(key, rclcpp::ParameterValue());
    }
  } else {
    // Set existing parameter
    if (isstring(argv[1])) {
      s_node->set_parameter(rclcpp::Parameter(key, string((char*)get_string(argv[1]))));
    } else if (isint(argv[1])) {
      s_node->set_parameter(rclcpp::Parameter(key, (int64_t)intval(argv[1])));
    } else if (isflt(argv[1])) {
      s_node->set_parameter(rclcpp::Parameter(key, (double)fltval(argv[1])));
    } else if (argv[1] == T) {
      s_node->set_parameter(rclcpp::Parameter(key, true));
    } else if (argv[1] == NIL) {
      s_node->set_parameter(rclcpp::Parameter(key, false));
    }
  }

  return (T);
}

pointer ROSEUS_GET_PARAM(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  numunion nu;
  string key;

  ckarg2(1, 2);
  if (isstring(argv[0])) key.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  if (!s_node->has_parameter(key)) {
    if (n == 2) {
      return COPYOBJ(ctx, 1, argv + 1);
    } else {
      RCLCPP_ERROR(s_node->get_logger(), "unknown ros parameter, key=%s", key.c_str());
      return NIL;
    }
  }

  rclcpp::Parameter param = s_node->get_parameter(key);
  pointer ret = NIL;

  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      ret = param.as_bool() ? T : NIL;
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      ret = makeint(param.as_int());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      ret = makeflt(param.as_double());
      break;
    case rclcpp::ParameterType::PARAMETER_STRING: {
      string s = param.as_string();
      ret = makestring((char*)s.c_str(), s.length());
    } break;
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      // Convert arrays to EusLisp lists
      string s = param.value_to_string();
      ret = makestring((char*)s.c_str(), s.length());
    } break;
    default:
      if (n == 2) {
        ret = COPYOBJ(ctx, 1, argv + 1);
      } else {
        ret = NIL;
      }
      break;
  }

  return ret;
}

pointer ROSEUS_HAS_PARAM(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string key;
  ckarg(1);
  if (isstring(argv[0])) key.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  return (s_node->has_parameter(key)) ? T : NIL;
}

pointer ROSEUS_DELETE_PARAM(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string key;
  ckarg(1);
  if (isstring(argv[0])) key.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  try {
    s_node->undeclare_parameter(key);
    return (T);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(s_node->get_logger(), "Failed to delete parameter %s: %s", key.c_str(), e.what());
    return (NIL);
  }
}

/***********************************************************
 *   Name resolution / Discovery
 ************************************************************/

pointer ROSEUS_RESOLVE_NAME(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(1);
  if (!isstring(argv[0])) error(E_NOSTRING);
  string src((char*)(argv[0]->c.str.chars));
  string dst = resolveName(src);
  return makestring((char*)dst.c_str(), dst.length());
}

pointer ROSEUS_GETNAME(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(0);
  string name = s_node->get_fully_qualified_name();
  return makestring((char*)name.c_str(), name.length());
}

pointer ROSEUS_GETNAMESPACE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(0);
  string ns = s_node->get_namespace();
  return makestring((char*)ns.c_str(), ns.length());
}

pointer ROSEUS_GET_TOPICS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(0);

  auto topics = s_node->get_topic_names_and_types();

  register pointer ret, first;
  ret = cons(ctx, NIL, NIL);
  first = ret;
  vpush(ret);
  for (const auto& entry : topics) {
    string name = entry.first;
    string type = entry.second.empty() ? "" : entry.second[0];
    pointer tmp = cons(ctx,
                       makestring((char*)name.c_str(), name.length()),
                       makestring((char*)type.c_str(), type.length()));
    vpush(tmp);
    ccdr(ret) = cons(ctx, tmp, NIL);
    ret = ccdr(ret);
    vpop();  // tmp
  }
  vpop();  // ret
  return ccdr(first);
}

pointer ROSEUS_GET_NODES(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(0);

  auto nodes = s_node->get_node_names();

  register pointer ret, first;
  ret = cons(ctx, NIL, NIL);
  first = ret;
  vpush(ret);
  for (const auto& node : nodes) {
    ccdr(ret) = cons(ctx, makestring((char*)node.c_str(), node.length()), NIL);
    ret = ccdr(ret);
  }
  vpop();  // ret
  return ccdr(first);
}

/***********************************************************
 *   Package utilities
 ************************************************************/

pointer ROSEUS_ROSPACK_FIND(register context* ctx, int n, pointer* argv) {
  ckarg(1);
  string pkg;
  if (isstring(argv[0])) pkg.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  try {
    string path = ament_index_cpp::get_package_share_directory(pkg);
    return makestring((char*)path.c_str(), path.length());
  } catch (const std::exception& e) {
    return (NIL);
  }
}

pointer ROSEUS_ROSPACK_DEPENDS(register context* ctx, int n, pointer* argv) {
  ckarg(1);
  RCLCPP_WARN(rclcpp::get_logger("roseus"),
              "rospack-depends is not supported in ROS 2. Use ament_index instead.");
  return (NIL);
}

pointer ROSEUS_ROSPACK_PLUGINS(register context* ctx, int n, pointer* argv) {
  ckarg(2);
  RCLCPP_WARN(rclcpp::get_logger("roseus"),
              "rospack-plugins is not supported in ROS 2.");
  return (NIL);
}

/***********************************************************
 *   Timer / Callback Group
 ************************************************************/

pointer ROSEUS_CREATE_TIMER(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  numunion nu;
  bool oneshot = false;
  string groupname;
  pointer fncallback = NIL, args;
  rclcpp::CallbackGroup::SharedPtr cbg = nullptr;
  string fncallname;
  float period = ckfltval(argv[0]);

  // period callbackfunc args0 ... argsN [:oneshot oneshot] [:groupname groupname]
  bool check_key = true;
  while (check_key) {
    if (n > 1 && issymbol(argv[n - 2])) {
      if (argv[n - 2] == K_ROSEUS_ONESHOT && issymbol(argv[n - 1])) {
        if (argv[n - 1] != NIL) {
          oneshot = true;
        }
        n -= 2;
      } else if (argv[n - 2] == K_ROSEUS_GROUPNAME && isstring(argv[n - 1])) {
        groupname.assign((char*)get_string(argv[n - 1]));
        auto it = s_mapCallbackGroup.find(groupname);
        if (it != s_mapCallbackGroup.end()) {
          RCLCPP_DEBUG(s_node->get_logger(), "create-timer with groupname=%s", groupname.c_str());
          cbg = it->second;
        } else {
          RCLCPP_ERROR(s_node->get_logger(),
                       "Groupname %s is missing. Call (ros::create-nodehandle \"%s\") first.",
                       groupname.c_str(), groupname.c_str());
          return (NIL);
        }
        n -= 2;
      } else {
        check_key = false;
      }
    } else {
      check_key = false;
    }
  }

  // Resolve callback function
  fncallback = argv[1];
  pointer scb = resolveCallback(ctx, fncallback);

  // Build function name for map key
  if (piscode(scb)) {
    std::ostringstream ss;
    ss << reinterpret_cast<long>(scb) << " ";
    for (int i = 2; i < n; i++) {
      if (issymbol(argv[i]))
        ss << string((char*)(argv[i]->c.sym.pname->c.str.chars)) << " ";
    }
    fncallname = ss.str();
  } else if (issymbol(scb)) {
    fncallname = string((char*)(scb->c.sym.pname->c.str.chars));
  } else {
    std::ostringstream ss;
    ss << reinterpret_cast<long>(scb);
    fncallname = ss.str();
  }

  // Arguments
  args = NIL;
  for (int i = n - 1; i >= 2; i--) args = cons(ctx, argv[i], args);

  protectCallback(ctx, fncallback, args);

  RCLCPP_DEBUG(s_node->get_logger(), "create timer %s at %f (oneshot=%d) (groupname=%s)",
               fncallname.c_str(), period, oneshot, groupname.c_str());

  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period));

  shared_ptr<rclcpp::TimerBase> timer;
  if (oneshot) {
    // For oneshot, we create a timer that cancels itself after firing
    timer = s_node->create_wall_timer(
        duration,
        [scb, args, fncallname]() {
          context* ctx = current_ctx;
          pointer argp = args;
          int argc = 0;

          if (!(issymbol(scb) || piscode(scb) || ccar(scb) == LAMCLOSURE)) {
            RCLCPP_ERROR(rclcpp::get_logger("roseus"), "can't find timer callback function");
            return;
          }

          while (argp != NIL) {
            ckpush(ccar(argp));
            argp = ccdr(argp);
            argc++;
          }
          ufuncall(ctx, (ctx->callfp ? ctx->callfp->form : NIL),
                   scb, (pointer)(ctx->vsp - argc), NULL, argc);
          while (argc-- > 0) vpop();

          // Cancel the timer (oneshot)
          auto it = s_mapTimered.find(fncallname);
          if (it != s_mapTimered.end()) {
            it->second->cancel();
          }
        },
        cbg);
  } else {
    timer = s_node->create_wall_timer(
        duration,
        [scb, args]() {
          context* ctx = current_ctx;
          pointer argp = args;
          int argc = 0;

          if (!(issymbol(scb) || piscode(scb) || ccar(scb) == LAMCLOSURE)) {
            RCLCPP_ERROR(rclcpp::get_logger("roseus"), "can't find timer callback function");
            return;
          }

          while (argp != NIL) {
            ckpush(ccar(argp));
            argp = ccdr(argp);
            argc++;
          }
          ufuncall(ctx, (ctx->callfp ? ctx->callfp->form : NIL),
                   scb, (pointer)(ctx->vsp - argc), NULL, argc);
          while (argc-- > 0) vpop();
        },
        cbg);
  }

  s_mapTimered[fncallname] = timer;
  return (T);
}

pointer ROSEUS_CREATE_NODEHANDLE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  string groupname;
  ckarg2(1, 2);

  if (isstring(argv[0])) groupname.assign((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  if (s_mapCallbackGroup.find(groupname) != s_mapCallbackGroup.end()) {
    RCLCPP_DEBUG(s_node->get_logger(), "groupname %s is already used", groupname.c_str());
    return (NIL);
  }

  auto cbg = s_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  s_mapCallbackGroup[groupname] = cbg;

  return (T);
}

/***********************************************************
 *   Action Client / Server helpers
 ************************************************************/

/* Parse "pkg/action/Name" → (pkg, Name) */
static bool parseActionType(const string& action_type,
                            string& pkg, string& name) {
  // Expected format: "pkg/action/Name"
  auto pos1 = action_type.find('/');
  if (pos1 == string::npos) return false;
  auto pos2 = action_type.find('/', pos1 + 1);
  if (pos2 == string::npos) return false;
  pkg = action_type.substr(0, pos1);
  name = action_type.substr(pos2 + 1);
  return true;
}

/* Load action type support dynamically */
static const rosidl_action_type_support_t*
loadActionTypeSupport(const string& action_type,
                      vector<shared_ptr<rcpputils::SharedLibrary>>& libs) {
  string pkg, name;
  if (!parseActionType(action_type, pkg, name)) return nullptr;

  // Load library
  auto lib = rclcpp::get_typesupport_library(
      action_type, "rosidl_typesupport_cpp");
  libs.push_back(lib);

  // Build symbol name
  string symbol = "rosidl_typesupport_cpp__get_action_type_support_handle__" + pkg + "__action__" + name;

  typedef const rosidl_action_type_support_t* (*GetTSFunc)();
  auto func = reinterpret_cast<GetTSFunc>(lib->get_symbol(symbol.c_str()));
  if (!func) return nullptr;
  return func();
}

/* Load service introspection for struct allocation */
static const rosidl_typesupport_introspection_cpp::ServiceMembers*
loadServiceIntrospection(const string& service_type,
                         vector<shared_ptr<rcpputils::SharedLibrary>>& libs) {
  auto lib = rclcpp::get_typesupport_library(
      service_type, "rosidl_typesupport_introspection_cpp");
  libs.push_back(lib);
  auto ts = rclcpp::get_service_typesupport_handle(
      service_type, "rosidl_typesupport_introspection_cpp", *lib);
  return static_cast<
      const rosidl_typesupport_introspection_cpp::ServiceMembers*>(ts->data);
}

/* Load message introspection for struct allocation */
static const rosidl_typesupport_introspection_cpp::MessageMembers*
loadMessageIntrospection(const string& msg_type,
                         vector<shared_ptr<rcpputils::SharedLibrary>>& libs) {
  auto lib = rclcpp::get_typesupport_library(
      msg_type, "rosidl_typesupport_introspection_cpp");
  libs.push_back(lib);
  auto ts = rclcpp::get_message_typesupport_handle(
      msg_type, "rosidl_typesupport_introspection_cpp", *lib);
  return static_cast<
      const rosidl_typesupport_introspection_cpp::MessageMembers*>(ts->data);
}

/* Serialize EusLisp message to C struct via CDR */
static bool eusMsgToCStruct(pointer eus_msg,
                            const rosidl_message_type_support_t* msg_ts,
                            const rosidl_typesupport_introspection_cpp::MessageMembers* intro,
                            vector<uint8_t>& buf) {
  buf.resize(intro->size_of_, 0);
  intro->init_function(buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rclcpp::SerializedMessage cdr = serializeEusMessage(eus_msg);
  rmw_ret_t rc = rmw_deserialize(
      &cdr.get_rcl_serialized_message(), msg_ts, buf.data());
  if (rc != RMW_RET_OK) {
    intro->fini_function(buf.data());
    return false;
  }
  return true;
}

/* Deserialize C struct to EusLisp message via CDR */
static bool cStructToEusMsg(const void* c_struct,
                            const rosidl_message_type_support_t* msg_ts,
                            pointer eus_msg) {
  rcl_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_serialized_message_init(&serialized, 256, &allocator);

  rmw_ret_t rc = rmw_serialize(c_struct, msg_ts, &serialized);
  if (rc != RMW_RET_OK) {
    rmw_serialized_message_fini(&serialized);
    return false;
  }

  auto sm = std::make_shared<rclcpp::SerializedMessage>(serialized.buffer_length);
  auto& rcl_msg = sm->get_rcl_serialized_message();
  memcpy(rcl_msg.buffer, serialized.buffer, serialized.buffer_length);
  rcl_msg.buffer_length = serialized.buffer_length;
  rmw_serialized_message_fini(&serialized);

  deserializeEusMessage(eus_msg, sm);
  return true;
}

/***********************************************************
 *   Action Client
 ************************************************************/

pointer ROSEUS_ACTION_CLIENT_INIT(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name, action_type;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  if (isstring(argv[1])) action_type.assign((char*)get_string(argv[1]));
  else error(E_NOSTRING);

  if (s_mapActionClients.find(action_name) != s_mapActionClients.end()) {
    RCLCPP_WARN(s_node->get_logger(), "action client %s already initialized", action_name.c_str());
    return (T);
  }

  string pkg, name;
  if (!parseActionType(action_type, pkg, name)) {
    RCLCPP_ERROR(s_node->get_logger(), "invalid action type format: %s", action_type.c_str());
    return (NIL);
  }

  auto data = make_shared<ActionClientData>();
  data->client = rcl_action_get_zero_initialized_client();

  // Load action type support
  data->action_ts = loadActionTypeSupport(action_type, data->libs);
  if (!data->action_ts) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to load action type support for %s", action_type.c_str());
    return (NIL);
  }

  // Load introspection for sub-types
  string goal_svc_type = pkg + "/action/" + name + "_SendGoal";
  string result_svc_type = pkg + "/action/" + name + "_GetResult";
  string cancel_svc_type = "action_msgs/srv/CancelGoal";
  string feedback_msg_type = pkg + "/action/" + name + "_FeedbackMessage";
  string status_msg_type = "action_msgs/msg/GoalStatusArray";

  try {
    data->goal_svc_intro = loadServiceIntrospection(goal_svc_type, data->libs);
    data->result_svc_intro = loadServiceIntrospection(result_svc_type, data->libs);
    data->cancel_svc_intro = loadServiceIntrospection(cancel_svc_type, data->libs);
    data->feedback_msg_intro = loadMessageIntrospection(feedback_msg_type, data->libs);
    data->status_msg_intro = loadMessageIntrospection(status_msg_type, data->libs);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to load introspection for %s: %s",
                 action_type.c_str(), e.what());
    return (NIL);
  }

  // Init action client
  rcl_action_client_options_t options = rcl_action_client_get_default_options();
  rcl_ret_t rc = rcl_action_client_init(
      &data->client,
      s_node->get_node_base_interface()->get_rcl_node_handle(),
      data->action_ts, action_name.c_str(), &options);

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to init action client %s: %s",
                 action_name.c_str(), rcl_get_error_string().str);
    rcl_reset_error();
    return (NIL);
  }

  s_mapActionClients[action_name] = data;
  return (T);
}

pointer ROSEUS_ACTION_CLIENT_DESTROY(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(1);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);

  rcl_action_client_fini(
      &it->second->client,
      s_node->get_node_base_interface()->get_rcl_node_handle());
  s_mapActionClients.erase(it);
  return (T);
}

pointer ROSEUS_ACTION_WAIT_FOR_SERVER(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  numunion nu;
  ckarg2(1, 2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  float timeout = -1;
  if (n > 1 && argv[1] != NIL) timeout = ckfltval(argv[1]);

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) {
    RCLCPP_ERROR(s_node->get_logger(), "action client %s not found", action_name.c_str());
    return (NIL);
  }

  auto start = std::chrono::steady_clock::now();
  while (true) {
    bool is_available = false;
    rcl_ret_t rc = rcl_action_server_is_available(
        s_node->get_node_base_interface()->get_rcl_node_handle(),
        &it->second->client, &is_available);
    if (rc == RCL_RET_OK && is_available) return (T);

    if (timeout >= 0) {
      auto elapsed = std::chrono::steady_clock::now() - start;
      if (std::chrono::duration<double>(elapsed).count() >= timeout)
        return (NIL);
    }
    if (!rclcpp::ok()) return (NIL);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  return (NIL);
}

pointer ROSEUS_ACTION_SEND_GOAL(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) {
    RCLCPP_ERROR(s_node->get_logger(), "action client %s not found", action_name.c_str());
    return (NIL);
  }
  auto& data = *it->second;

  // EusLisp SendGoal_Request → C struct
  auto req_members = data.goal_svc_intro->request_members_;
  vector<uint8_t> req_buf;
  vpush(eus_request);
  if (!eusMsgToCStruct(eus_request,
                       data.action_ts->goal_service_type_support->request_typesupport,
                       req_members, req_buf)) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to serialize goal request for %s", action_name.c_str());
    vpop();
    return (NIL);
  }
  vpop();

  int64_t seq_num = 0;
  rcl_ret_t rc = rcl_action_send_goal_request(
      &data.client, req_buf.data(), &seq_num);
  req_members->fini_function(req_buf.data());

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to send goal request for %s: %s",
                 action_name.c_str(), rcl_get_error_string().str);
    rcl_reset_error();
    return (NIL);
  }

  return makeint(seq_num);
}

pointer ROSEUS_ACTION_TAKE_GOAL_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.goal_svc_intro->response_members_;
  vector<uint8_t> resp_buf(resp_members->size_of_, 0);
  resp_members->init_function(resp_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_request_id_t header;
  memset(&header, 0, sizeof(header));
  rcl_ret_t rc = rcl_action_take_goal_response(&data.client, &header, resp_buf.data());

  if (rc != RCL_RET_OK) {
    resp_members->fini_function(resp_buf.data());
    return (NIL);
  }

  vpush(eus_response);
  bool ok = cStructToEusMsg(resp_buf.data(),
                            data.action_ts->goal_service_type_support->response_typesupport,
                            eus_response);
  vpop();
  resp_members->fini_function(resp_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_SEND_RESULT_REQUEST(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  auto req_members = data.result_svc_intro->request_members_;
  vector<uint8_t> req_buf;
  vpush(eus_request);
  if (!eusMsgToCStruct(eus_request,
                       data.action_ts->result_service_type_support->request_typesupport,
                       req_members, req_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  int64_t seq_num = 0;
  rcl_ret_t rc = rcl_action_send_result_request(
      &data.client, req_buf.data(), &seq_num);
  req_members->fini_function(req_buf.data());

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to send result request for %s", action_name.c_str());
    rcl_reset_error();
    return (NIL);
  }
  return makeint(seq_num);
}

pointer ROSEUS_ACTION_TAKE_RESULT_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.result_svc_intro->response_members_;
  vector<uint8_t> resp_buf(resp_members->size_of_, 0);
  resp_members->init_function(resp_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_request_id_t header;
  memset(&header, 0, sizeof(header));
  rcl_ret_t rc = rcl_action_take_result_response(&data.client, &header, resp_buf.data());

  if (rc != RCL_RET_OK) {
    resp_members->fini_function(resp_buf.data());
    return (NIL);
  }

  vpush(eus_response);
  bool ok = cStructToEusMsg(resp_buf.data(),
                            data.action_ts->result_service_type_support->response_typesupport,
                            eus_response);
  vpop();
  resp_members->fini_function(resp_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_TAKE_FEEDBACK(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_feedback = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  vector<uint8_t> fb_buf(data.feedback_msg_intro->size_of_, 0);
  data.feedback_msg_intro->init_function(
      fb_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rcl_ret_t rc = rcl_action_take_feedback(&data.client, fb_buf.data());
  if (rc != RCL_RET_OK) {
    data.feedback_msg_intro->fini_function(fb_buf.data());
    return (NIL);
  }

  vpush(eus_feedback);
  bool ok = cStructToEusMsg(fb_buf.data(),
                            data.action_ts->feedback_message_type_support, eus_feedback);
  vpop();
  data.feedback_msg_intro->fini_function(fb_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_TAKE_STATUS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_status = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  vector<uint8_t> status_buf(data.status_msg_intro->size_of_, 0);
  data.status_msg_intro->init_function(
      status_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rcl_ret_t rc = rcl_action_take_status(&data.client, status_buf.data());
  if (rc != RCL_RET_OK) {
    data.status_msg_intro->fini_function(status_buf.data());
    return (NIL);
  }

  vpush(eus_status);
  bool ok = cStructToEusMsg(status_buf.data(),
                            data.action_ts->status_message_type_support, eus_status);
  vpop();
  data.status_msg_intro->fini_function(status_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_SEND_CANCEL(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  auto req_members = data.cancel_svc_intro->request_members_;
  vector<uint8_t> req_buf;
  vpush(eus_request);
  if (!eusMsgToCStruct(eus_request,
                       data.action_ts->cancel_service_type_support->request_typesupport,
                       req_members, req_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  int64_t seq_num = 0;
  rcl_ret_t rc = rcl_action_send_cancel_request(
      &data.client, req_buf.data(), &seq_num);
  req_members->fini_function(req_buf.data());

  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return (NIL);
  }
  return makeint(seq_num);
}

pointer ROSEUS_ACTION_TAKE_CANCEL_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionClients.find(action_name);
  if (it == s_mapActionClients.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.cancel_svc_intro->response_members_;
  vector<uint8_t> resp_buf(resp_members->size_of_, 0);
  resp_members->init_function(resp_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_request_id_t header;
  memset(&header, 0, sizeof(header));
  rcl_ret_t rc = rcl_action_take_cancel_response(&data.client, &header, resp_buf.data());

  if (rc != RCL_RET_OK) {
    resp_members->fini_function(resp_buf.data());
    return (NIL);
  }

  vpush(eus_response);
  bool ok = cStructToEusMsg(resp_buf.data(),
                            data.action_ts->cancel_service_type_support->response_typesupport,
                            eus_response);
  vpop();
  resp_members->fini_function(resp_buf.data());
  return ok ? T : NIL;
}

/***********************************************************
 *   Action Server
 ************************************************************/

pointer ROSEUS_ACTION_SERVER_INIT(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name, action_type;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  if (isstring(argv[1])) action_type.assign((char*)get_string(argv[1]));
  else error(E_NOSTRING);

  if (s_mapActionServers.find(action_name) != s_mapActionServers.end()) {
    RCLCPP_WARN(s_node->get_logger(), "action server %s already initialized", action_name.c_str());
    return (T);
  }

  string pkg, name_str;
  if (!parseActionType(action_type, pkg, name_str)) {
    RCLCPP_ERROR(s_node->get_logger(), "invalid action type format: %s", action_type.c_str());
    return (NIL);
  }

  auto data = make_shared<ActionServerData>();
  data->server = rcl_action_get_zero_initialized_server();
  memset(&data->last_goal_request_header, 0, sizeof(rmw_request_id_t));
  memset(&data->last_result_request_header, 0, sizeof(rmw_request_id_t));
  memset(&data->last_cancel_request_header, 0, sizeof(rmw_request_id_t));

  data->action_ts = loadActionTypeSupport(action_type, data->libs);
  if (!data->action_ts) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to load action type support for %s", action_type.c_str());
    return (NIL);
  }

  string goal_svc_type = pkg + "/action/" + name_str + "_SendGoal";
  string result_svc_type = pkg + "/action/" + name_str + "_GetResult";
  string cancel_svc_type = "action_msgs/srv/CancelGoal";
  string feedback_msg_type = pkg + "/action/" + name_str + "_FeedbackMessage";
  string status_msg_type = "action_msgs/msg/GoalStatusArray";

  try {
    data->goal_svc_intro = loadServiceIntrospection(goal_svc_type, data->libs);
    data->result_svc_intro = loadServiceIntrospection(result_svc_type, data->libs);
    data->cancel_svc_intro = loadServiceIntrospection(cancel_svc_type, data->libs);
    data->feedback_msg_intro = loadMessageIntrospection(feedback_msg_type, data->libs);
    data->status_msg_intro = loadMessageIntrospection(status_msg_type, data->libs);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to load introspection for %s: %s",
                 action_type.c_str(), e.what());
    return (NIL);
  }

  // Get a clock for the action server
  rcl_clock_t* clock = s_node->get_clock()->get_clock_handle();

  rcl_action_server_options_t options = rcl_action_server_get_default_options();
  rcl_ret_t rc = rcl_action_server_init(
      &data->server,
      s_node->get_node_base_interface()->get_rcl_node_handle(),
      clock, data->action_ts, action_name.c_str(), &options);

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to init action server %s: %s",
                 action_name.c_str(), rcl_get_error_string().str);
    rcl_reset_error();
    return (NIL);
  }

  s_mapActionServers[action_name] = data;
  return (T);
}

pointer ROSEUS_ACTION_SERVER_DESTROY(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(1);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);

  // Goal handles are managed by rcl_action_server internally;
  // rcl_action_server_fini will clean them up.

  rcl_action_server_fini(
      &it->second->server,
      s_node->get_node_base_interface()->get_rcl_node_handle());
  s_mapActionServers.erase(it);
  return (T);
}

pointer ROSEUS_ACTION_TAKE_GOAL_REQUEST(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto req_members = data.goal_svc_intro->request_members_;
  vector<uint8_t> req_buf(req_members->size_of_, 0);
  req_members->init_function(req_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rcl_ret_t rc = rcl_action_take_goal_request(
      &data.server, &data.last_goal_request_header, req_buf.data());

  if (rc != RCL_RET_OK) {
    req_members->fini_function(req_buf.data());
    return (NIL);
  }

  vpush(eus_request);
  bool ok = cStructToEusMsg(req_buf.data(),
                            data.action_ts->goal_service_type_support->request_typesupport,
                            eus_request);
  vpop();
  req_members->fini_function(req_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_SEND_GOAL_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.goal_svc_intro->response_members_;
  vector<uint8_t> resp_buf;
  vpush(eus_response);
  if (!eusMsgToCStruct(eus_response,
                       data.action_ts->goal_service_type_support->response_typesupport,
                       resp_members, resp_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  rcl_ret_t rc = rcl_action_send_goal_response(
      &data.server, &data.last_goal_request_header, resp_buf.data());
  resp_members->fini_function(resp_buf.data());

  if (rc != RCL_RET_OK) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to send goal response for %s", action_name.c_str());
    rcl_reset_error();
    return (NIL);
  }
  return (T);
}

pointer ROSEUS_ACTION_ACCEPT_NEW_GOAL(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_uuid = argv[1];  // EusLisp string (16 bytes)

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  rcl_action_goal_info_t goal_info;
  memset(&goal_info, 0, sizeof(goal_info));
  if (isstring(eus_uuid) && strlength(eus_uuid) >= 16) {
    memcpy(goal_info.goal_id.uuid, get_string(eus_uuid), 16);
  }
  // Set stamp
  rclcpp::Time now = s_node->now();
  goal_info.stamp.sec = static_cast<int32_t>(now.seconds());
  goal_info.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);

  rcl_action_goal_handle_t* gh = rcl_action_accept_new_goal(
      &data.server, &goal_info);
  if (!gh) {
    RCLCPP_ERROR(s_node->get_logger(), "failed to accept new goal for %s", action_name.c_str());
    return (NIL);
  }

  // Track the goal handle
  data.goal_handles.push_back(gh);

  // Transition to executing state
  rcl_action_update_goal_state(gh, GOAL_EVENT_EXECUTE);

  // Publish status update
  rcl_action_goal_status_array_t status_msg =
      rcl_action_get_zero_initialized_goal_status_array();
  rcl_action_get_goal_status_array(&data.server, &status_msg);
  rcl_action_publish_status(&data.server, &status_msg.msg);
  rcl_action_goal_status_array_fini(&status_msg);

  return makeint(data.goal_handles.size() - 1);  // return handle index
}

pointer ROSEUS_ACTION_PUBLISH_FEEDBACK(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_feedback = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  vector<uint8_t> fb_buf;
  vpush(eus_feedback);
  if (!eusMsgToCStruct(eus_feedback,
                       data.action_ts->feedback_message_type_support,
                       data.feedback_msg_intro, fb_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  rcl_ret_t rc = rcl_action_publish_feedback(&data.server, fb_buf.data());
  data.feedback_msg_intro->fini_function(fb_buf.data());

  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return (NIL);
  }
  return (T);
}

pointer ROSEUS_ACTION_PUBLISH_STATUS(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(1);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  rcl_action_goal_status_array_t status_msg =
      rcl_action_get_zero_initialized_goal_status_array();
  rcl_ret_t rc = rcl_action_get_goal_status_array(&data.server, &status_msg);
  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return (NIL);
  }

  rc = rcl_action_publish_status(&data.server, &status_msg.msg);
  rcl_action_goal_status_array_fini(&status_msg);

  return (rc == RCL_RET_OK) ? T : NIL;
}

pointer ROSEUS_ACTION_TAKE_RESULT_REQUEST(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto req_members = data.result_svc_intro->request_members_;
  vector<uint8_t> req_buf(req_members->size_of_, 0);
  req_members->init_function(req_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rcl_ret_t rc = rcl_action_take_result_request(
      &data.server, &data.last_result_request_header, req_buf.data());

  if (rc != RCL_RET_OK) {
    req_members->fini_function(req_buf.data());
    return (NIL);
  }

  vpush(eus_request);
  bool ok = cStructToEusMsg(req_buf.data(),
                            data.action_ts->result_service_type_support->request_typesupport,
                            eus_request);
  vpop();
  req_members->fini_function(req_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_SEND_RESULT_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.result_svc_intro->response_members_;
  vector<uint8_t> resp_buf;
  vpush(eus_response);
  if (!eusMsgToCStruct(eus_response,
                       data.action_ts->result_service_type_support->response_typesupport,
                       resp_members, resp_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  rcl_ret_t rc = rcl_action_send_result_response(
      &data.server, &data.last_result_request_header, resp_buf.data());
  resp_members->fini_function(resp_buf.data());

  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return (NIL);
  }
  return (T);
}

pointer ROSEUS_ACTION_TAKE_CANCEL_REQUEST(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_request = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto req_members = data.cancel_svc_intro->request_members_;
  vector<uint8_t> req_buf(req_members->size_of_, 0);
  req_members->init_function(req_buf.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  rcl_ret_t rc = rcl_action_take_cancel_request(
      &data.server, &data.last_cancel_request_header, req_buf.data());

  if (rc != RCL_RET_OK) {
    req_members->fini_function(req_buf.data());
    return (NIL);
  }

  vpush(eus_request);
  bool ok = cStructToEusMsg(req_buf.data(),
                            data.action_ts->cancel_service_type_support->request_typesupport,
                            eus_request);
  vpop();
  req_members->fini_function(req_buf.data());
  return ok ? T : NIL;
}

pointer ROSEUS_ACTION_SEND_CANCEL_RESPONSE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(2);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  pointer eus_response = argv[1];

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  auto resp_members = data.cancel_svc_intro->response_members_;
  vector<uint8_t> resp_buf;
  vpush(eus_response);
  if (!eusMsgToCStruct(eus_response,
                       data.action_ts->cancel_service_type_support->response_typesupport,
                       resp_members, resp_buf)) {
    vpop();
    return (NIL);
  }
  vpop();

  rcl_ret_t rc = rcl_action_send_cancel_response(
      &data.server, &data.last_cancel_request_header, resp_buf.data());
  resp_members->fini_function(resp_buf.data());

  if (rc != RCL_RET_OK) {
    rcl_reset_error();
    return (NIL);
  }
  return (T);
}

pointer ROSEUS_ACTION_UPDATE_GOAL_STATE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(3);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);
  int handle_idx = ckintval(argv[1]);
  int goal_event = ckintval(argv[2]);

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);
  auto& data = *it->second;

  if (handle_idx < 0 || handle_idx >= (int)data.goal_handles.size()) return (NIL);

  rcl_ret_t rc = rcl_action_update_goal_state(
      data.goal_handles[handle_idx],
      static_cast<rcl_action_goal_event_t>(goal_event));

  return (rc == RCL_RET_OK) ? T : NIL;
}

pointer ROSEUS_ACTION_NOTIFY_GOAL_DONE(register context* ctx, int n, pointer* argv) {
  isInstalledCheck;
  ckarg(1);
  string action_name;
  if (isstring(argv[0])) action_name = resolveName((char*)get_string(argv[0]));
  else error(E_NOSTRING);

  auto it = s_mapActionServers.find(action_name);
  if (it == s_mapActionServers.end()) return (NIL);

  rcl_ret_t rc = rcl_action_notify_goal_done(&it->second->server);
  return (rc == RCL_RET_OK) ? T : NIL;
}

/***********************************************************
 *   Stub functions (ROS 1 compatibility)
 ************************************************************/

pointer ROSEUS_GET_HOST(register context* ctx, int n, pointer* argv) {
  ckarg(0);
  RCLCPP_WARN(rclcpp::get_logger("roseus"),
              "get-host is not supported in ROS 2 (no master)");
  return makestring((char*)"", 0);
}

pointer ROSEUS_GET_PORT(register context* ctx, int n, pointer* argv) {
  ckarg(0);
  RCLCPP_WARN(rclcpp::get_logger("roseus"),
              "get-port is not supported in ROS 2 (no master)");
  return makeint(0);
}

pointer ROSEUS_GET_URI(register context* ctx, int n, pointer* argv) {
  ckarg(0);
  RCLCPP_WARN(rclcpp::get_logger("roseus"),
              "get-uri is not supported in ROS 2 (no master)");
  return makestring((char*)"", 0);
}

/***********************************************************
 *   Module initialization: ___roseus
 ************************************************************/

extern pointer K_FUNCTION_DOCUMENTATION;

pointer ___roseus(register context* ctx, int n, pointer* argv, pointer env) {
  pointer rospkg, p = Spevalof(PACKAGE);
  rospkg = findpkg(makestring("ROS", 3));
  if (rospkg == 0) rospkg = makepkg(ctx, makestring("ROS", 3), NIL, NIL);
  Spevalof(PACKAGE) = rospkg;

  // Define variables (backward compatible with ROS 1)
  QANON = defvar(ctx, "*ANONYMOUS-NAME*", makeint(1), rospkg);
  QNOOUT = defvar(ctx, "*NO-ROSOUT*", makeint(2), rospkg);
  QROSDEBUG = defvar(ctx, "*ROSDEBUG*", makeint(1), rospkg);
  QROSINFO = defvar(ctx, "*ROSINFO*", makeint(2), rospkg);
  QROSWARN = defvar(ctx, "*ROSWARN*", makeint(3), rospkg);
  QROSERROR = defvar(ctx, "*ROSERROR*", makeint(4), rospkg);
  QROSFATAL = defvar(ctx, "*ROSFATAL*", makeint(5), rospkg);

  // Version info (used by roseus.l for startup message)
  {
    const char* ver = ROSEUS_VERSION_STR;
    pointer l = makestring((char*)ver, strlen(ver));
    vpush(l);
    l = stacknlist(ctx, 1);
    QREPOVERSION = defvar(ctx, "ROSEUS-REPO-VERSION", l, rospkg);
  }

  // Core functions
  defun(ctx, "ROSEUS-RAW", argv[0], (pointer(*)())ROSEUS, "");
  defun(ctx, "SPIN", argv[0], (pointer(*)())ROSEUS_SPIN, "Enter simple event loop");
  defun(ctx, "SPIN-ONCE", argv[0], (pointer(*)())ROSEUS_SPINONCE,
        "&optional groupname  ;; spin only group\n\n"
        "Process a single round of callbacks.\n");
  defun(ctx, "TIME-NOW-RAW", argv[0], (pointer(*)())ROSEUS_TIME_NOW, "");
  defun(ctx, "RATE", argv[0], (pointer(*)())ROSEUS_RATE,
        "frequency\n\nConstruct ros timer for periodic sleeps");
  defun(ctx, "SLEEP", argv[0], (pointer(*)())ROSEUS_SLEEP,
        "Sleeps for any leftover time in a cycle.");
  defun(ctx, "DURATION-SLEEP", argv[0], (pointer(*)())ROSEUS_DURATION_SLEEP,
        "second\n\nSleeps for amount of the time specified by this duration.");
  defun(ctx, "OK", argv[0], (pointer(*)())ROSEUS_OK,
        "Check whether it's time to exit.");

  // Logging
  defun(ctx, "ROS-DEBUG", argv[0], (pointer(*)())ROSEUS_ROSDEBUG,
        "write message to debug output");
  defun(ctx, "ROS-INFO", argv[0], (pointer(*)())ROSEUS_ROSINFO,
        "write message to info output");
  defun(ctx, "ROS-WARN", argv[0], (pointer(*)())ROSEUS_ROSWARN,
        "write message to warn output");
  defun(ctx, "ROS-ERROR", argv[0], (pointer(*)())ROSEUS_ROSERROR,
        "write message to error output");
  defun(ctx, "ROS-FATAL", argv[0], (pointer(*)())ROSEUS_ROSFATAL,
        "write message to fatal output");
  defun(ctx, "SET-LOGGER-LEVEL", argv[0], (pointer(*)())ROSEUS_SET_LOGGER_LEVEL, "");
  defun(ctx, "EXIT", argv[0], (pointer(*)())ROSEUS_EXIT, "Exit ros client");

  // Pub/Sub
  defun(ctx, "SUBSCRIBE", argv[0], (pointer(*)())ROSEUS_SUBSCRIBE,
        "topicname message_type callbackfunc args0 ... argsN "
        "&optional (queuesize 1) &key (:groupname groupname)\n\n"
        "Subscribe to a topic.\n");
  defun(ctx, "UNSUBSCRIBE", argv[0], (pointer(*)())ROSEUS_UNSUBSCRIBE,
        "topicname\n\nUnsubscribe topic");
  defun(ctx, "ADVERTISE", argv[0], (pointer(*)())ROSEUS_ADVERTISE,
        "topic message_class &optional (queuesize 1) (latch nil)\n\n"
        "Advertise a topic.\n");
  defun(ctx, "UNADVERTISE", argv[0], (pointer(*)())ROSEUS_UNADVERTISE,
        "Unadvertise topic");
  defun(ctx, "PUBLISH", argv[0], (pointer(*)())ROSEUS_PUBLISH,
        "topic message\n\nPublish a message on the topic\n");
  defun(ctx, "GET-NUM-PUBLISHERS", argv[0], (pointer(*)())ROSEUS_GETNUMPUBLISHERS,
        "Returns the number of publishers on a topic.");
  defun(ctx, "GET-NUM-SUBSCRIBERS", argv[0], (pointer(*)())ROSEUS_GETNUMSUBSCRIBERS,
        "Returns the number of subscribers on a topic.");

  // Services
  defun(ctx, "WAIT-FOR-SERVICE", argv[0], (pointer(*)())ROSEUS_WAIT_FOR_SERVICE,
        "servicename &optional timeout\n\n"
        "Wait for a service to be advertised and available.");
  defun(ctx, "SERVICE-EXISTS", argv[0], (pointer(*)())ROSEUS_SERVICE_EXISTS,
        "servicename\n\nChecks if a service is advertised.");
  defun(ctx, "SERVICE-CALL", argv[0], (pointer(*)())ROSEUS_SERVICE_CALL,
        "servicename message_type &optional persist\n\nInvoke RPC service\n");
  defun(ctx, "ADVERTISE-SERVICE", argv[0], (pointer(*)())ROSEUS_ADVERTISE_SERVICE,
        "servicename message_type callbackfunc args0 ... argsN "
        "&key (:groupname groupname)\n\nAdvertise a service\n");
  defun(ctx, "UNADVERTISE-SERVICE", argv[0], (pointer(*)())ROSEUS_UNADVERTISE_SERVICE,
        "Unadvertise service");

  // Parameters
  defun(ctx, "SET-PARAM", argv[0], (pointer(*)())ROSEUS_SET_PARAM,
        "key value\n\nSet parameter");
  defun(ctx, "GET-PARAM", argv[0], (pointer(*)())ROSEUS_GET_PARAM,
        "key\n\nGet parameter");
  defun(ctx, "HAS-PARAM", argv[0], (pointer(*)())ROSEUS_HAS_PARAM,
        "Check whether a parameter exists.");
  defun(ctx, "DELETE-PARAM", argv[0], (pointer(*)())ROSEUS_DELETE_PARAM,
        "key\n\nDelete parameter");

  // Package utilities
  defun(ctx, "ROSPACK-FIND", argv[0], (pointer(*)())ROSEUS_ROSPACK_FIND,
        "Returns ros package share directory path");
  defun(ctx, "ROSPACK-DEPENDS", argv[0], (pointer(*)())ROSEUS_ROSPACK_DEPENDS,
        "Returns ros package dependencies list (stub in ROS 2)");
  defun(ctx, "ROSPACK-PLUGINS", argv[0], (pointer(*)())ROSEUS_ROSPACK_PLUGINS,
        "Returns plugins of ros packages (stub in ROS 2)");

  // Name resolution / Discovery
  defun(ctx, "RESOLVE-NAME", argv[0], (pointer(*)())ROSEUS_RESOLVE_NAME,
        "Returns ros resolved name");
  defun(ctx, "GET-NAME", argv[0], (pointer(*)())ROSEUS_GETNAME,
        "Returns current node name");
  defun(ctx, "GET-NAMESPACE", argv[0], (pointer(*)())ROSEUS_GETNAMESPACE,
        "Returns current node namespace");
  defun(ctx, "GET-TOPICS", argv[0], (pointer(*)())ROSEUS_GET_TOPICS,
        "Get the list of topics.");
  defun(ctx, "GET-NODES", argv[0], (pointer(*)())ROSEUS_GET_NODES,
        "Get the list of nodes.");

  // Timer / Callback Group
  defun(ctx, "CREATE-TIMER", argv[0], (pointer(*)())ROSEUS_CREATE_TIMER,
        "period callbackfunc args0 ... argsN "
        "&key (:oneshot oneshot) (:groupname groupname)\n\n"
        "Create periodic callbacks.\n");
  defun(ctx, "CREATE-NODEHANDLE", argv[0], (pointer(*)())ROSEUS_CREATE_NODEHANDLE,
        "groupname &optional namespace\n\nCreate callback group with given group name.\n");

  // Action Client
  defun(ctx, "ACTION-CLIENT-INIT", argv[0], (pointer(*)())ROSEUS_ACTION_CLIENT_INIT,
        "action-name action-type\n\nInitialize action client.\n");
  defun(ctx, "ACTION-CLIENT-DESTROY", argv[0], (pointer(*)())ROSEUS_ACTION_CLIENT_DESTROY,
        "action-name\n\nDestroy action client.\n");
  defun(ctx, "ACTION-WAIT-FOR-SERVER", argv[0], (pointer(*)())ROSEUS_ACTION_WAIT_FOR_SERVER,
        "action-name &optional timeout\n\nWait for action server to be available.\n");
  defun(ctx, "ACTION-SEND-GOAL", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_GOAL,
        "action-name goal-request\n\nSend goal request. Returns sequence number.\n");
  defun(ctx, "ACTION-TAKE-GOAL-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_GOAL_RESPONSE,
        "action-name response-instance\n\nTake goal response.\n");
  defun(ctx, "ACTION-SEND-RESULT-REQUEST", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_RESULT_REQUEST,
        "action-name result-request\n\nSend result request.\n");
  defun(ctx, "ACTION-TAKE-RESULT-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_RESULT_RESPONSE,
        "action-name response-instance\n\nTake result response.\n");
  defun(ctx, "ACTION-TAKE-FEEDBACK", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_FEEDBACK,
        "action-name feedback-instance\n\nTake feedback message.\n");
  defun(ctx, "ACTION-TAKE-STATUS", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_STATUS,
        "action-name status-instance\n\nTake status array.\n");
  defun(ctx, "ACTION-SEND-CANCEL", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_CANCEL,
        "action-name cancel-request\n\nSend cancel request.\n");
  defun(ctx, "ACTION-TAKE-CANCEL-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_CANCEL_RESPONSE,
        "action-name response-instance\n\nTake cancel response.\n");

  // Action Server
  defun(ctx, "ACTION-SERVER-INIT", argv[0], (pointer(*)())ROSEUS_ACTION_SERVER_INIT,
        "action-name action-type\n\nInitialize action server.\n");
  defun(ctx, "ACTION-SERVER-DESTROY", argv[0], (pointer(*)())ROSEUS_ACTION_SERVER_DESTROY,
        "action-name\n\nDestroy action server.\n");
  defun(ctx, "ACTION-TAKE-GOAL-REQUEST", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_GOAL_REQUEST,
        "action-name request-instance\n\nTake goal request (server side).\n");
  defun(ctx, "ACTION-SEND-GOAL-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_GOAL_RESPONSE,
        "action-name response\n\nSend goal response (server side).\n");
  defun(ctx, "ACTION-ACCEPT-NEW-GOAL", argv[0], (pointer(*)())ROSEUS_ACTION_ACCEPT_NEW_GOAL,
        "action-name uuid-string\n\nAccept new goal. Returns handle index.\n");
  defun(ctx, "ACTION-PUBLISH-FEEDBACK", argv[0], (pointer(*)())ROSEUS_ACTION_PUBLISH_FEEDBACK,
        "action-name feedback-msg\n\nPublish feedback (server side).\n");
  defun(ctx, "ACTION-PUBLISH-STATUS", argv[0], (pointer(*)())ROSEUS_ACTION_PUBLISH_STATUS,
        "action-name\n\nPublish status (server side).\n");
  defun(ctx, "ACTION-TAKE-RESULT-REQUEST", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_RESULT_REQUEST,
        "action-name request-instance\n\nTake result request (server side).\n");
  defun(ctx, "ACTION-SEND-RESULT-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_RESULT_RESPONSE,
        "action-name response\n\nSend result response (server side).\n");
  defun(ctx, "ACTION-TAKE-CANCEL-REQUEST", argv[0], (pointer(*)())ROSEUS_ACTION_TAKE_CANCEL_REQUEST,
        "action-name request-instance\n\nTake cancel request (server side).\n");
  defun(ctx, "ACTION-SEND-CANCEL-RESPONSE", argv[0], (pointer(*)())ROSEUS_ACTION_SEND_CANCEL_RESPONSE,
        "action-name response\n\nSend cancel response (server side).\n");
  defun(ctx, "ACTION-UPDATE-GOAL-STATE", argv[0], (pointer(*)())ROSEUS_ACTION_UPDATE_GOAL_STATE,
        "action-name handle-index goal-event\n\nUpdate goal state.\n");
  defun(ctx, "ACTION-NOTIFY-GOAL-DONE", argv[0], (pointer(*)())ROSEUS_ACTION_NOTIFY_GOAL_DONE,
        "action-name\n\nNotify that goal is done.\n");

  // Stubs for ROS 1 compatibility
  defun(ctx, "GET-HOST", argv[0], (pointer(*)())ROSEUS_GET_HOST,
        "Get the hostname where the master runs. (stub in ROS 2)");
  defun(ctx, "GET-PORT", argv[0], (pointer(*)())ROSEUS_GET_PORT,
        "Get the port where the master runs. (stub in ROS 2)");
  defun(ctx, "GET-URI", argv[0], (pointer(*)())ROSEUS_GET_URI,
        "Get the full URI to the master. (stub in ROS 2)");

  pointer_update(Spevalof(PACKAGE), p);

  return 0;
}
