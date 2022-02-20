deploy-all: deploy-far_blue deploy-far_red

deploy-%: %.auto deploy-%-trajectory
	adb push $< /sdcard/FIRST/auto/$<

deploy-%-trajectory: %.json
	adb push $< /sdcard/FIRST/trajectories/$<