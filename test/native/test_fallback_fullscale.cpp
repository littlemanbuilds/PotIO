#include <PotIO.h>
#include "../support/MiniTest.h"

#include <initializer_list>
#include <type_traits>

// Compile this translation unit with multiple fallback declarations. Readers
// without kFullScale must get the same validation as readers that declare it.
struct FallbackReader
{
    int operator()() const noexcept { return 65536; }
};

struct ValidFallbackReader
{
    int operator()() const noexcept { return 512; }
};

template <typename Device>
static void check_device(bool expectedValid)
{
    Device defaults;
    const typename Device::Config cfg{};
    Device configured(cfg);
    for (Device *device : {&defaults, &configured})
    {
        device->update(10u, 0.01f);
        const auto &status = device->state().status;
        CHECK(status.valid == expectedValid);
        CHECK(status.has_value == expectedValid);
        CHECK(status.error == (expectedValid ? PotIO::ReadError::None : PotIO::ReadError::InvalidConfiguration));
        CHECK(status.sequence == (expectedValid ? 1u : 0u));
        CHECK(status.sample_ms == (expectedValid ? 10u : 0u));
    }
}

int main()
{
    MiniTest::begin("fallback reader range is validated before raw storage in both constructors");
    constexpr bool validRange = POTIO_DEFAULT_FULLSCALE > 0 && POTIO_DEFAULT_FULLSCALE <= UINT16_MAX;
    using Reader = typename std::conditional<validRange, ValidFallbackReader, FallbackReader>::type;
    check_device<PotIO::LinearPot<Reader>>(validRange);
    check_device<PotIO::ContinuousPot<Reader>>(validRange);
    check_device<PotIO::SteppedPot<5u, Reader>>(validRange);
    check_device<PotIO::Joystick2D<Reader, Reader>>(validRange);
    return 0;
}
