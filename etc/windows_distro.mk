TMPDIR = /tmp/openmrn_windows_distro
#LIBTARGETS = freertos.armv7m \
#             freertos.armv6m
LIBTARGETS = linux.x86
SRCDIRS = console cue dcc executor freertos_drivers/common nmranet os utils

windows_distro: openmrn_windows_distro.zip

CP = cp -rf

openmrn_windows_distro.zip:
	rm -rf $(TMPDIR)
	mkdir -p $(TMPDIR)/targets
	$(CP) $(OPENMRNPATH)/include $(TMPDIR)
	$(foreach dir,$(SRCDIRS),mkdir -p $(TMPDIR)/include/$(dir); $(CP) $(OPENMRNPATH)/src/$(dir)/*.h $(TMPDIR)/include/$(dir) 2> /dev/null; $(CP) $(OPENMRNPATH)/src/$(dir)/*.hxx $(TMPDIR)/include/$(dir) 2> /dev/null;)
	$(foreach dir,$(LIBTARGETS),mkdir -p $(TMPDIR)/targets/$(dir)/lib; $(CP) $(OPENMRNPATH)/targets/$(dir)/lib/*.a $(TMPDIR)/targets/$(dir)/lib;)
	touch $(OPENMRNPATH)/openmrn_windows_distro.zip
	
	

windows_distro_clean:
	rm -rf openmrn_windows_distro.zip

