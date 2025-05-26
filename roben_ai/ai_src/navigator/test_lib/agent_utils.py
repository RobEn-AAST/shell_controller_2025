from ai_src.carla_others.agents.navigation.behavior_agent import BehaviorAgent


def set_agent(ego_vehicle):
    agent = BehaviorAgent(ego_vehicle, behavior="aggressive")  # cautious, normal, aggressive
    agent.ignore_traffic_lights(True)
    agent._behavior.max_speed = 47

    return agent