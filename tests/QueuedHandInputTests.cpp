#include "physics-interaction/input/QueuedHandInput.h"

#include <cassert>

int main()
{
    using namespace rock::queued_hand_input;
    Records records;
    const Origin right{Hand::Right, 1, 27};
    const Origin left{Hand::Left, 2, 27};
    constexpr std::uintptr_t movie = 0x100000, secondMovie = 0x200000;
    constexpr std::uintptr_t first = movie + 0x268, second = first + 0x50;

    // Delayed, interleaved dispatch must follow its own event, not the last
    // trigger. A second delivery must never repeat the consumed attribution.
    assert(records.replace(first, movie, right));
    assert(records.replace(second, movie, left));
    assert(records.take(first, movie) == right);
    assert(records.take(second, movie) == left);
    assert(records.take(first, movie).hand == Hand::None);

    // Native ring wrap overwrites an old event with untagged mouse/keyboard
    // input. Neither that input nor a later key may inherit the old hand.
    assert(records.replace(first, movie, right));
    records.discard(first);
    assert(records.take(first, movie).hand == Hand::None);
    assert(records.replace(first, movie, right));
    assert(records.replace(first, movie, left));
    assert(records.take(first, movie) == left);

    // A destroyed movie may be recreated at the same address. Another live
    // movie must keep its attribution when the first is retired.
    assert(records.replace(first, movie, right));
    assert(records.replace(secondMovie + 0x268, secondMovie, left));
    records.retire(movie);
    assert(records.take(first, movie).hand == Hand::None);
    assert(records.take(secondMovie + 0x268, secondMovie) == left);

    // Reject mismatched owners and consume the stale record, including reuse
    // of an entry by a different movie before a late dispatch arrives.
    assert(records.replace(first, movie, right));
    assert(records.take(first, secondMovie).hand == Hand::None);
    assert(records.take(first, movie).hand == Hand::None);

    // Capacity exhaustion cannot evict pending attribution or infer a hand.
    for (std::size_t i = 0; i < Records::capacity; ++i)
        assert(records.replace(first + i * 0x50, movie, right));
    assert(!records.replace(0x400000, secondMovie, left));
    for (std::size_t i = 0; i < Records::capacity; ++i)
        assert(records.take(first + i * 0x50, movie) == right);
    assert(records.take(0x400000, secondMovie).hand == Hand::None);
    assert(records.replace(first, movie, right));
    assert(!records.replace(first, movie, {}));
    assert(records.take(first, movie).hand == Hand::None);
}
