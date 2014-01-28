.PHONY: clean All

All:
	@echo ----------Building project:[ VISIONdevelopment - Debug ]----------
	@"$(MAKE)" -f "VISIONdevelopment.mk"
clean:
	@echo ----------Cleaning project:[ VISIONdevelopment - Debug ]----------
	@"$(MAKE)" -f "VISIONdevelopment.mk" clean
