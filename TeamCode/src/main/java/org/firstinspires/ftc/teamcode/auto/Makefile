deploy-all: deploy-far_blue deploy-far_red deploy-near_red deploy-test

deploy-%: %.auto deploy-%-trajectory
	@echo deploing $%
	@adb push $< /sdcard/FIRST/auto/$<
	@adb push $< /sdcard/FIRST/trajectories/$<