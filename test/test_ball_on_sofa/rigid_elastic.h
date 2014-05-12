#ifndef __CJ_RIGID_ELASTIC_H__
#define __CJ_RIGID_ELASTIC_H__

#include "FullStVKSimulator.h"
#include "SubspaceSimulator.h"
#include "rigid_body.h"
#include "shell_deformer/deformer.h"

class rigid_elastic {














protected :
    std::shared_ptr<SIMULATOR::Simulator>   elastic_solid_;
    std::shared_ptr<deformer>               shell_;
    std::shared_ptr<rigid_bodys>             rigid_;
};





















#endif
