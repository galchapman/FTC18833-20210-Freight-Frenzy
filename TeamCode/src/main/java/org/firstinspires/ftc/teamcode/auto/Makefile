deploy-%: %.auto deploy-%-trajectory
	adb push $< /sdcard/FIRST/auto/$<

deploy-%-trajectory: %.json
	adb push $< /sdcard/FIRST/trajectories/$<