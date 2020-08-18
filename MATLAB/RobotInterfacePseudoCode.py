

BeforeStart:

	Set variables 

	Connect Loop
	
	return Connected
	
Start:

	While !Shutdown:
	
		get instruction_id
		
		if instruction_id == HALT
			
			Shutdown := true
			
		if instruction_id == INST
		
			get instruction
			
			display warning message with instruction
			
		if instruction_id == POSE
		
			return robot pose
			
		if instruction_id == MOVE
			
			movec
			
			movej % joint space
			
			movel
			
			movep
		
			
			get move_pose
			
			return START
			
			move
			
			return STOP
			
		if instruction_id == WAIT
		
			get wait_time
			
			wait for wait_time
			
		
			
			
		
			
