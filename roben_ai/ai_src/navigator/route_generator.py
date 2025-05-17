from ai_src.carla_others.agents.navigation.global_route_planner import GlobalRoutePlanner



def generate_full_route(start_loc, targets, grp: GlobalRoutePlanner):
    """
    returnrs list of [waypoint, roundoff]
    can access locaation throguh full_route[0][0].transform.location.x, .y, .z
    """
    full_route = []
    current_start = start_loc

    for target in targets:
        # Get route segment
        segment = grp.trace_route(current_start, target)

        if not segment:
            print(f"⚠️ Couldn't find path to {target}! Skipping...")
            continue

        # Append to full route (skip first waypoint if not first segment)
        full_route += segment if not full_route else segment[1:]
        current_start = target  # Update start for next segment

    return full_route


def generate_simple_path(start_loc, targets, grp: GlobalRoutePlanner):
    """
    returns list of locations, access with full_path[0].x, .y, .z
    """
    full_path = []
    current_start = start_loc

    for target in targets:
        segment = grp.trace_route(current_start, target)
        if not segment:
            continue

        # Extract just the locations
        locations = [wp[0].transform.location for wp in (segment if not full_path else segment[1:])]
        full_path += locations
        current_start = target

    return full_path