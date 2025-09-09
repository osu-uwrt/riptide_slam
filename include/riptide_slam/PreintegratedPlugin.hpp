#pragma once

#include "BasePlugin.hpp"

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <mutex>

namespace riptide_slam {
namespace plugin {

class PreintegratedPlugin : public BasePlugin {

public:

    size_t plugin_type_hash() override {
        return typeid(PreintegratedPlugin).hash_code();
    }

    /*!
     *  @brief Gets Factor created from this Sensors data
     *
     *  The data that has been recieved since the creation of the previous
     *  factor is used to constrain new Key(s) to previous Key(s) that were
     *  created from this class.
     *
     *  @param[in] idx Index of newly created key
     *
     *  @returns NonlinearFactor constraining X(idx) to X(previous_idx)
     *           where X represents some key(s).
     */
    virtual gtsam::NonlinearFactor::shared_ptr getFactor(uint64_t idx) = 0;

protected:
    std::mutex mutex;   //! Preintegrated plugins interact with data using multiple threads

};

}
}
