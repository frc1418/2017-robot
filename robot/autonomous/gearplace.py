from .base_auto import VictisAuto

from components import swervedrive, gearpicker, shooter
from controllers import angle_controller, pos_controller, position_history, position_tracker

from magicbot.state_machine import timed_state, state, AutonomousStateMachine
from magicbot.magic_tunable import tunable

class MiddleGearPlace(VictisAuto):
    '''
    This state is meant the be extended and should never be run by itself 
    '''
    MODE_NAME = "Middle Gear Place"
    DEFUALT = False
    DISABLED = True
    
    # Injection
    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    shooter = shooter.Shooter
    
    x_ctrl = pos_controller.XPosController
    y_ctrl = pos_controller.YPosController
    
    fc_y_ctrl = pos_controller.FCYPosController
    fc_x_ctrl = pos_controller.FCXPosController
    
    angle_ctrl = angle_controller.AngleController
    moving_angle_ctrl = angle_controller.MovingAngleController
    
    tracker = position_tracker.PositionTracker
    fc_tracker = position_tracker.FCPositionTracker
    
    m_direction = tunable(1, subtable='middle') # When direction is 1 the shooting tower is left of the robot
    
    # Distances
    # Warning: this is a field centric mode all positions are relative to starting position
    m_out_y = tunable(6.3, subtable='middle')
    m_out_x = tunable(0, subtable='middle')
    
    m_back_y = tunable(4, subtable='middle') # The cord that the robot backs off to before strafeing
    
    # Shooting position
    m_tower_y = tunable(3, subtable='middle')
    m_tower_x = tunable(-4, subtable='middle')
    m_tower_angle = tunable(55, subtable='middle')
    
    def initialize(self):
        pass
    
    #################################################
    # This portion of the code is for placing gears #
    #################################################
    
    @state
    def middle_start(self):
        self.drive.field_centric = True
        self.gear_picker._picker_state = 2
        self.angle_ctrl.reset_angle()
        self.fc_tracker.enable()
        
        self.next_state('middle_drive_to_gear')
        
    @timed_state(duration=4, next_state='middle_try_place')
    def middle_drive_to_gear(self):
        self.fc_x_ctrl.move_to(self.m_out_x)
        self.fc_y_ctrl.move_to(self.m_out_y)
        self.moving_angle_ctrl.align_to(0)
        
        if self.fc_y_ctrl.is_at_location():
            self.gear_picker._picker_state = 1
            self.next_state('middle_drive_back')
            
    @timed_state(duration=1, next_state='middle_drive_back')        
    def middle_try_place(self):
        self.fc_x_ctrl.move_to(self.m_out_x)
        self.fc_y_ctrl.move_to(self.m_out_y)
        self.drive.set_raw_rcw(0.4)
        
        if self.fc_y_ctrl.is_at_location():
            self.next_state('middle_drive_back')
            
    @timed_state(duration = 3, next_state='failed')
    def middle_drive_back(self, initial_call):
        if initial_call:
            self.gear_picker._picker_state = 1
            
        self.fc_y_ctrl.move_to(self.m_back_y)
        self.moving_angle_ctrl.align_to(0)
        
        if self.fc_y_ctrl.is_at_location():
            self.next_state('transition')
            
    ############################################ 
    # This portion of the code is for shooting #
    ############################################
    
    @state
    def middle_start_shoot(self):
        self.next_state('middle_to_tower')
    
    @timed_state(duration = 5, next_state='failed')
    def middle_to_tower(self):
        self.fc_y_ctrl.move_to(self.m_tower_y)
        self.fc_x_ctrl.move_to(self.m_tower_x * self.m_direction)
        self.moving_angle_ctrl.align_to(self.m_tower_angle * self.m_direction)
        
        self.shooter.force_spin()
        
        if self.fc_x_ctrl.is_at_location() and self.fc_y_ctrl.is_at_location():
            self.next_state('middle_align_to_tower')
            
    @timed_state(duration = 5, next_state='failed')
    def middle_align_to_tower(self):
        self.angle_ctrl.align_to(self.m_tower_angle * self.m_direction)
        
        self.shooter.force_spin()
        
        if self.angle_ctrl.is_aligned():
            self.next_state('middle_verify_tower_angle')
            
    @timed_state(duration = 0.5, next_state='middle_shoot')
    def middle_verify_tower_angle(self):
        
        self.shooter.force_spin()
        
        if not self.angle_ctrl.is_aligned_to(self.m_tower_angle * self.m_direction):
            self.next_state('middle_align_to_tower')
    
    @state
    def middle_shoot(self):
        self.shooter.force_spin()
        self.shooter.force_feed()
    
    ######################################################
    # This portion of the code is used when not shooting #
    ######################################################
    
    @state
    def middle_end(self):
        self.next_state('finish')
        
class SideGearPlace(VictisAuto):
    '''
    This state is meant the be extended and should never be run by itself 
    '''
    MODE_NAME = "Side Gear Place"
    DEFUALT = False
    DISABLED = True

    # Injection
    drive = swervedrive.SwerveDrive
    gear_picker = gearpicker.GearPicker
    shooter = shooter.Shooter
    
    x_ctrl = pos_controller.XPosController
    y_ctrl = pos_controller.YPosController
    
    fc_y_ctrl = pos_controller.FCYPosController
    fc_x_ctrl = pos_controller.FCXPosController
    
    angle_ctrl = angle_controller.AngleController
    moving_angle_ctrl = angle_controller.MovingAngleController
    
    tracker = position_tracker.PositionTracker
    fc_tracker = position_tracker.FCPositionTracker
    
    s_direction = tunable(1, subtable='side') # When set to 1 this will run gear place on the right side
    
    # Distances
    # Warning: this is a field centric mode all positions are relative to starting position
    s_out_y = tunable(7.5, subtable='side')
    s_out_x = tunable(0, subtable='side')
    s_out_angle = tunable(-60, subtable='side')
    
    s_to_peg_distance = tunable(2, subtable='side')
    
    s_peg_y = tunable(9.2, subtable='side') # TODO: Check this math (coverted from non field centric auto)
    s_peg_x = tunable(-1, subtable='side')
    
    s_back_y = tunable(7.5, subtable='side')
    s_back_x = tunable(0, subtable='side')
    
    s_tower_y = tunable(5.7, subtable='side')
    s_tower_x = tunable(1, subtable='side')
    s_tower_angle = tunable(-40, subtable='side')
    
    def initialize(self):
        pass
    
    #################################################
    # This portion of the code is for placing gears #
    #################################################
    
    @state
    def side_start(self):
        self.drive.field_centric = True
        self.gear_picker._picker_state = 2
        self.angle_ctrl.reset_angle()
        self.fc_tracker.enable()
        
        self.next_state('side_drive_to_gear')
        
    @timed_state(duration = 7, next_state='failed')
    def side_drive_to_gear(self):
        self.fc_y_ctrl.move_to(self.s_out_y)
        self.fc_x_ctrl.move_to(self.s_out_x)
        
        if self.fc_tracker.get_y() > self.s_out_y/2:
            self.moving_angle_ctrl.align_to(self.s_out_angle * self.s_direction)
        
        if self.fc_y_ctrl.is_at_location() and self.fc_x_ctrl.is_at_location():
            self.next_state('side_align_to_peg')
    
    @timed_state(duration = 7, next_state='failed')
    def side_align_to_peg(self):
        self.angle_ctrl.align_to(self.s_out_angle * self.s_direction)
        
        if self.angle_ctrl.is_aligned():
            self.next_state('side_verify_peg_alignment')
    
    @timed_state(duration = 0.5, next_state='side_drive_to_peg')
    def side_verify_peg_alignment(self):
        if not self.angle_ctrl.is_aligned_to(self.s_out_angle * self.s_direction):
            self.next_state('side_align_to_peg')
    
    @timed_state(duration=3, next_state='side_try_place')
    def side_drive_to_peg(self, initial_call):
        if initial_call:
            self.tracker.enable()
            self.tracker.reset()
            
        self.y_ctrl.move_to(self.s_to_peg_distance)
        self.moving_angle_ctrl.align_to(self.s_out_angle * self.s_direction)
        
        if self.y_ctrl.is_at_location():
            self.next_state('side_drive_back')
            
    @timed_state(duration=1, next_state='side_drive_back')
    def side_try_place(self):
        self.y_ctrl.move_to(self.s_to_peg_distance)
        self.drive.set_raw_rcw(-0.4 * self.s_direction)
        
        if self.y_ctrl.is_at_location():
            self.next_state('side_drive_back')
    
    @timed_state(duration=5, next_state='failed')
    def side_drive_back(self, initial_call):
        if initial_call:
            self.tracker.reset()
            self.gear_picker._picker_state = 1
        
        self.y_ctrl.move_to(-self.s_to_peg_distance)
        self.moving_angle_ctrl.align_to(self.s_out_angle * self.s_direction)
        
        if self.y_ctrl.is_at_location():
            self.next_state('transition')
    
    ############################################ 
    # This portion of the code is for shooting #
    ############################################

    @state
    def side_start_shoot(self):
        self.next_state('side_to_tower')
        
    @timed_state(duration=5, next_state='failed')
    def side_to_tower(self):
        self.fc_y_ctrl.move_to(self.s_tower_y)
        self.fc_x_ctrl.move_to(self.s_tower_x * self.s_direction)
        self.moving_angle_ctrl.align_to(self.s_tower_angle * self.s_direction)
        
        self.shooter.force_spin()
        
        if self.fc_y_ctrl.is_at_location() and self.fc_x_ctrl.is_at_location():
            self.next_state('side_align_to_tower')
    
    @timed_state(duration = 7, next_state='failed')
    def side_align_to_tower(self):
        self.angle_ctrl.align_to(self.s_tower_angle * self.s_direction)
        
        self.shooter.force_spin()
        
        if self.angle_ctrl.is_aligned():
            self.next_state('side_verify_tower_alignment')
    
    @timed_state(duration = 0.5, next_state='side_shoot')
    def side_verify_tower_alignment(self):
        
        self.shooter.force_spin()
        
        if not self.angle_ctrl.is_aligned_to(self.s_tower_angle * self.s_direction):
            self.next_state('side_align_to_tower')  
    
    @state
    def side_shoot(self):
        self.shooter.force_spin()
        self.shooter.force_feed()   
    
    ######################################################
    # This portion of the code is used when not shooting #
    ######################################################
    
    @state
    def side_end(self):
        self.next_state('finish')
        
class GearPlace(MiddleGearPlace, SideGearPlace):
    MODE_NAME = "Gear Place"
    DEFAULT = True
    DISABLED = False
    
    
    angle_ctrl = angle_controller.AngleController
    moving_angle_ctrl = angle_controller.MovingAngleController
    
    x_ctrl = pos_controller.XPosController
    y_ctrl = pos_controller.YPosController
    
    gear_picker = gearpicker.GearPicker
    
    # Position tunable should be one of four things
    # 'middle_left' - middle gear place with left boiler
    # 'middle_right' - middle gear place with right boiler 
    # 'left' - left side gear place
    # 'right' - right side gear place
    position = tunable('middle_left')
    
    shoot = tunable(True) #
    
    def initialize(self):
        MiddleGearPlace.initialize(self)
        SideGearPlace.initialize(self)
    
    @state(first = True)
    def first_state(self):
        self.gear_picker._picker_state = 2
        
        if self.position == 'right':
            self.s_direction = 1
            self.position = 'side'
        elif self.position == 'left':
            self.s_direction = -1
            self.position = 'side'
        elif self.position == 'middle_left':
            self.m_direction = 1
            self.position = 'middle'
        elif self.position == 'middle_right':
            self.m_direction = -1
            self.position = 'middle'
        
        #print(self._StateMachine__states)
        
        print('Starting ', self.position, ' gear place')
        
        self.next_state(self.position + '_start')
    
    @state
    def transition(self):
        if self.shoot:
            self.next_state(self.position + '_start_shoot')
        else:
            self.next_state(self.position + '_end')
            
    @state
    def failed(self):
        self.drive.debug(debug_modules = True)
        self.next_state('finish')
        
    @state
    def finish(self):
        self.drive.flush()
        self.done()
        
    

    