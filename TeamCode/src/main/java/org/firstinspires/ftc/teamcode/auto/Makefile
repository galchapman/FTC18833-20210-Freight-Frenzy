deploy-all: deploy-far_blue deploy-far_red deploy-near_red deploy-test

deploy-%: push-%.auto push-%.json
	

push-%.auto: %.auto
	@adb push $< /sdcard/FIRST/auto/$<

push-%.json: %.json
	@adb push $< /sdcard/FIRST/trajectories/$<