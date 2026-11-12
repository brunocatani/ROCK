#include "setup-wizard/SetupWizard.h"

// The configuration provider's tests run without RPS UI: the wizard is never available.
namespace rock::setup_wizard
{
    bool requestOpen() noexcept { return false; }
}
