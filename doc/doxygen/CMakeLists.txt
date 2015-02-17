remake_set(IMAGE_PATH 
  ${CMAKE_CURRENT_SOURCE_DIR}/figures)
remake_set(HTML_EXTRA_FILES 
  ${CMAKE_CURRENT_SOURCE_DIR}/../cheatsheet/cheatsheet_latest.pdf)

remake_add_documentation(
  DOXYGEN Doxyfile.remake
  MAIN_PAGE mainpage.dox
  INPUT ${CMAKE_CURRENT_SOURCE_DIR} ${REMAKE_PROJECT_SOURCE_DIR}
)
