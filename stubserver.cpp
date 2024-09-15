
#include "stubserver.h"
#include <jsonrpccpp/server/connectors/httpserver.h>

using namespace jsonrpc;

class JSONServer : public stubserver {
public:
  stubserver(AbstractServerConnector &connector, serverVersion_t type);

  virtual void setParam(const Json::Value &args);
  virtual Json::Value getParam();
};
