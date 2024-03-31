#include "hdmap_engine/common/status.h"

namespace hdmap {

Status::Status() : error_code_(ErrorCode::kOk), msg_("ok") {}

}  // namespace hdmap
