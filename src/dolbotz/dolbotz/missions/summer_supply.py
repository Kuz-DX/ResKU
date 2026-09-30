"""Compatibility import for the supply detector now provided by vision."""

from vision.supply import (
    SupplyNode,
    is_within_base_x_stop_distance,
    lookup_transform_with_latest_fallback,
    main,
)

ArmPickupNode = SupplyNode

if __name__ == '__main__':
    main()
