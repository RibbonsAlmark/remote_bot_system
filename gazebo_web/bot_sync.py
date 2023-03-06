import time
import rclpy
from rclpy.context import Context
from get_entity_state import EntityStateGetter
from set_entity_state import EntityStateSetter


if __name__ == "__main__":
    
    rclpy.init()
    
    context = Context()
    context.init(args=[], domain_id=1)
    Getter = EntityStateGetter("get_model_state_client", context)
    
    Setter = EntityStateSetter("set_model_state_client")
    
    while True:
        entity_state = Getter.get_entity_state("fishbot")
        entity_state.name = "fishbot"
        entity_state.reference_frame = "world"
        if entity_state  is not None:
            success = Setter.set_entity_state(entity_state)
        time.sleep(1)
        
    rclpy.shutdown()