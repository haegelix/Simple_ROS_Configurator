valid_project_name = RegExp("[a-z0-9_]{3,30}")

class DialogNewProject {
    is_duplicate(project_name){

    }

    open() {
        let project_name = prompt("Please give me name for your new project.\n\n" +
            "It may have 3 to 30 letters and only contain \n" +
            "- lowercase letters\n" +
            "- digits\n" +
            "- underscores\n", "project_name")

        if (!project_name.match(valid_project_name) || this.is_duplicate(project_name)) {
            alert("This name didn't work out...\n" +
                "- duplicate?\n" +
                "- wrong name?\n")
            return
        }

        let new_project = get_empty_ros2ui_project()
        new_project.project_info.package_name = project_name
        project = new_project
        save_package()
        request_package_list()
        select_package(project_name + ".json")
    }
}

dialog_new_project = new DialogNewProject()