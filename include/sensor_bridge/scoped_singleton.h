#pragma once

#include <memory>
#include <mutex>

#include "never_destroyed.h"

namespace drake {

/**
 * Provides thread-safe, global-safe access to a shared resource. When
 * all references are gone, the resource will be freed due to using a weak_ptr.
 * @tparam T Class of the resource. Must be default-constructible.
 * @tparam Unique Optional class, meant to make a unique specialization, such
 * that you can have multiple singletons of T if necessary.
 *
 * @note An example application is obtaining license for multiple disjoint
 * solver objects, where acquiring a license requires network communication,
 * and the solver is under an interface where you cannot explicitly pass the
 * license resource to the solver without violating encapsulation.
 */
template <typename T, typename Unique = void>
std::shared_ptr<T> GetScopedSingleton() {
  // Confine implementation to a class.
  class Singleton {
   public:
    Singleton() {}
    Singleton(const Singleton&) = delete;
    void operator=(const Singleton&) = delete;
    Singleton(Singleton&&) = delete;
    void operator=(Singleton&&) = delete;

    /*
     * Acquire a reference to resource if no other instance exists.
     * Otherwise, return a shared reference to the resource.
     */
    std::shared_ptr<T> Acquire() {
      // Acquiring and releasing the resource via a centralized weak pointer,
      // will be thread-safe. All constructions of T for this particular
      // Singleton are assigned to this same weak_ref_; the empty-check,
      // allocation, and weak_ref_ assignment are all guarded by the same
      // critical section.
      auto instance = weak_ref_.lock();
      if (!instance) {
        instance = std::make_shared<T>();
        weak_ref_ = instance;
      }
      return instance;
    }

   private:
    std::weak_ptr<T> weak_ref_;
  };
  // Allocate singleton as a static function local to control initialization.
  static never_destroyed<Singleton> singleton;
  return singleton.access().Acquire();
}

}  // namespace drake
