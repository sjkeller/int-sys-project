set(vcProjectFile "${RunCMake_TEST_BINARY_DIR}/foo.csproj")
if(NOT EXISTS "${vcProjectFile}")
  set(RunCMake_TEST_FAILED "Project file ${vcProjectFile} does not exist.")
  return()
endif()

set(startupObjectSet FALSE)

file(STRINGS "${vcProjectFile}" lines)
foreach(line IN LISTS lines)
  if(line MATCHES "^ *<StartupObject[^>]*>([^<>]+)</StartupObject>$")
    if("${CMAKE_MATCH_1}" STREQUAL "MyCompany.Package.MyStarterClass")
        message(STATUS "foo.csproj has StartupObject class set")
        set(startupObjectSet TRUE)
    endif()
  endif()
endforeach()

if(NOT startupObjectSet)
  set(RunCMake_TEST_FAILED "StartupObject not found or not set correctly.")
  return()
endif()
