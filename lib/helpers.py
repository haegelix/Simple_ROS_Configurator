def acceptable_answer_str(answer: str, acceptable: [str], use_upper=True) -> bool:
    """
    Checks if a given answer is in the range of acceptable answers
    :param answer: Answer to search for
    :param acceptable: acceptable answers given as array
    :param use_upper: Shall this method use the upper() method to ignore casing of letters?
    :return: boolean value says if answer is in the range of acceptables
    """
    if type(acceptable) == str:
        acceptable = [acceptable]
    if use_upper:
        return answer.upper() in list(map(str.upper, acceptable))
    else:
        return answer in acceptable


def acceptable_answer_int(answer: int, acceptable: [int]) -> bool:
    """
    Checks if a given answer is in the range of acceptable answers
    :param answer: Answer to search for
    :param acceptable: acceptable answers given as array
    :return: boolean value says if answer is in the range of acceptables
    """
    return answer in acceptable


def modify_and_copy_python_file(_paths, _package_info, templ_name: str, dest_name: str, replacements: [(str, str)]):
    # read template file
    in_file = open(_paths.get_template_path(templ_name), "r")
    py_code = in_file.read()
    in_file.close()

    # insert code-snippet
    for (old, new) in replacements:
        py_code = py_code.replace(old, new)

    # write file
    _paths.switch_to_package_py_dir(_package_info.pkg_config.package_name)
    out_file = open(dest_name + ".py", "w")
    out_file.write(py_code)
    out_file.close()


def register_entry_points(pkg_info: packageinfo.PackageConfig, entries: [EntryPoint]):
    """
    TODO doc
    """
    cwd = os.getcwd()
    setup_py_path = paths.get_package_src_path(pkg_info.package_name, "setup.py")

    logging.debug("setup.py path: " + str(setup_py_path))
    logging.debug("Adding entry_point(s): " + str(entries))
    logging.debug("DEBUG: " + str(paths.get_package_src_path(pkg_info.package_name)))

    # read template file
    in_file = open(setup_py_path, "r")
    py_code = in_file.read()
    in_file.close()

    # insert snippet
    for e in entries:
        py_code = py_code.replace("'console_scripts': [", "'console_scripts': [\n" + e.to_python() + ",")

    # write file
    out_file = open(setup_py_path, "w")
    out_file.write(py_code)
    out_file.close()

    # reset cwd
    os.chdir(cwd)


def clear_console():
    """
    Clears the terminal.
    TODO delete?
    """
    warnings.warn("This method is not usable in PyCharm Remote Env", DeprecationWarning, 2)
    command = 'clear'
    if os.name in ('nt', 'dos'):  # If Machine is running on Windows, use cls
        command = 'cls'
    os.system(command)
