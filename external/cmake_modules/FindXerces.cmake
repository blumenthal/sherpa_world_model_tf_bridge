#
# Find the native Xerces includes and library
#
# XERCES_INCLUDE_DIR - where to find dom/dom.hpp, etc.
# XERCES_LIBRARIES   - List of fully qualified libraries to link against when using Xerces.
# XERCES_FOUND       - Do not attempt to use Xerces if "no" or undefined.

FIND_PATH(XERCES_INCLUDE_DIR xercesc/dom/DOM.hpp
  /usr/local/include
  /usr/include
  $ENV{XERCESDIR}/include
  
)
#MESSAGE("XERCES_INCLUDE_DIR: " ${XERCES_INCLUDE_DIR})
# There may be some API changes between Xerces 1.x and 2.x
# I'm not sure how to deal with that in a .cmake file
# Perhaps it should be up to the application to figure out the version and
# API specifics from macros set in the headers?

FIND_LIBRARY(XERCES_LIBRARIES
  NAMES
    xerces-c
    xerces-c_3
    xerces-c_3D
    xerces-c_2
    xerces-c_2D
    xerces-c_1
    xerces-c_1D
  PATHS
    /usr/lib
    /usr/local/lib
    $ENV{XERCESDIR}/lib
)
#MESSAGE("libxerces:" ${XERCES_LIBRARIES})

# FIND_PATH(XERCES_LINK_DIR xerces-c_3D
    # /usr/local/lib
    # /usr/lib
	# $ENV{XERCESDIR}/lib  
# )

IF(XERCES_INCLUDE_DIR)
  IF(XERCES_LIBRARY)
    SET( XERCES_LIBRARIES ${XERCES_LIBRARIES} )
    SET( XERCES_FOUND "YES" )
	MESSAGE(STATUS "Xerces found ...")
  ENDIF(XERCES_LIBRARY)
ENDIF(XERCES_INCLUDE_DIR)
