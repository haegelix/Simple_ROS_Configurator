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
