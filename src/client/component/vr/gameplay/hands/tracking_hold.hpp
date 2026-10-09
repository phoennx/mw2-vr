#pragma once
#include "../../controller_input.hpp"

namespace vr::gameplay::hands
{
	// Presentation-only bridge for brief per-hand tracking dropouts. A support
	// controller occluded behind the rear hand while aiming would otherwise make
	// the whole IK solve fall back to the flat native viewmodel for a frame.
	// Gameplay ownership never reads this: it keeps the producer's validity.
	class tracking_hold
	{
	  public:
		inline static constexpr auto window = std::chrono::milliseconds(250);
		void apply(controller_input::frame& input) noexcept
		{
			for (unsigned h = 0; h < 2; ++h)
			{
				auto& held = hands_[h];
				if (input.grip[h].valid && input.aim[h].valid)
				{
					held = {input.grip[h], input.aim[h], input.sampled_at, input.reference_generation, true};
					continue;
				}
				if (held.valid && held.reference == input.reference_generation &&
				    input.sampled_at >= held.at && input.sampled_at - held.at <= window)
				{
					input.grip[h] = held.grip;
					input.aim[h] = held.aim;
				}
				else
					held.valid = false;
			}
		}
		void reset() noexcept { hands_ = {}; }

	  private:
		struct hand_state
		{
			controller_input::hand_pose grip{}, aim{};
			controller_input::clock::time_point at{};
			std::uint64_t reference{};
			bool valid{};
		};
		std::array<hand_state, 2> hands_{};
	};
} // namespace vr::gameplay::hands
