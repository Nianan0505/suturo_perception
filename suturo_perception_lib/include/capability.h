#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perceived_object.h"

namespace suturo_perception_lib
{
  class Capability
  {
    public:
      // The PerceivedObject that will be modified by the capability
      void setInputPerceivedObject(PerceivedObject &obj) {perceivedObject = obj;};
      /* The execute method that has to be implemented by the deriving classes.
       * This should modify the perceivedObject that was set before.
       */
      virtual void execute() = 0;

    protected:
      suturo_perception_lib::PerceivedObject perceivedObject;
  };  
}

#endif
