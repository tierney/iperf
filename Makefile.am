AUTOMAKE_OPTIONS = foreign
SUBDIRS = compat doc include src man

AM_CPPFLAGS = @STRIP_BEGIN@ \
	      -I$(top_srcdir)/include \
	      -I$(top_builddir)/include \
	      @STRIP_END@

AM_CXXFLAGS = -Wall
AM_FLAGS = -Wall

EXTRA_DIST = tests/reverse-matrix.sh tests/reverse-cases.tsv docs/iperf3-strategy.md

check-local:
	LEGACY_IPERF_BIN=$(top_builddir)/src/iperf \
	RUN_LEGACY=1 RUN_IPERF3=0 FAIL_ON_MISSING_BIN=1 $(srcdir)/tests/reverse-matrix.sh
