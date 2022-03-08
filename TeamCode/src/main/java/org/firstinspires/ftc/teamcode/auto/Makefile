deploy-all: deploy-far_blue deploy-far_red deploy-near_red deploy-test

deploy-%: %.auto %.json
	@adb push $%.auto /sdcard/FIRST/auto/$%.auto
	@adb push $%.json /sdcard/FIRST/trajectories/$%.json