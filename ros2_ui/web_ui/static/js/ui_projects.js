let socket = io("/package")
let packages = []
const package_selection_invalid = $("#package_selection_invalid")
const package_selection_dropdown = $("#package_selection")

// SETUP START
document.addEventListener("DOMContentLoaded", function () {
    socket.on('connect', function () {
        request_package_list()

        const queryString = window.location.search;
        const urlParams = new URLSearchParams(queryString);
        if (urlParams.has('project')) {
            const p = urlParams.get("project")
            request_package(p)
        }
    });

    socket.on('packages_list', function (message) {
        request_packages_list_success(message.list)
        const queryString = window.location.search;
        const urlParams = new URLSearchParams(queryString);
        if (urlParams.has('project')) {
            const p = urlParams.get("project")
            package_selection_dropdown.val(p)
        }
    });

    socket.on('package', function (message) {
        request_package_success(message.data)
    })

    socket.on('build', function (message) {
        const dialog = $('#build_dialog_log')
        dialog.html(dialog.html() + "<br>" + message.toString().replaceAll("\n", "<br>"))
    })
})

$(".pkg_data").on("input", function (event) {
    project.project_info.description = $("#pkg_description").val()
    project.project_info.version = $("#pkg_version").val()
    project.project_info.license = $("#pkg_license").val()
    project.project_info.maintainer = $("#pkg_maintainer").val()
    project.project_info.maintainer_mail = $("#pkg_maintainer_mail").val()
    console.log("pkg-data changed")
})

// SETUP END


function request_packages_list_success(list) {
    const curr_select = package_selection_dropdown.val()
    package_selection_dropdown.empty()

    package_selection_dropdown.append(package_selection_invalid)

    for (let filename of list) {
        let option = document.createElement("option")
        option.value = filename
        option.innerText = filename
        package_selection_dropdown.append(option)
    }

    if (list.includes(curr_select)) {
        package_selection_dropdown.val(curr_select)
    } else {
        package_selection_dropdown.val("invalid")
    }
}


/**
 *
 * @param package_name {string}
 */
function select_package(package_name) {
    const searchParams = new URLSearchParams()
    searchParams.append("project", package_name)
    goToSearchString(searchParams.toString())
}

function request_package_list() {
    socket.emit('get_packages_list')
}

function request_package(filename) {
    socket.emit('get_package', {filename: filename})
}

function save_package() {
    socket.emit('save_package', {data: project})
}

/**
 * Variable to house the current project
 * @type {ros2_ui_project}
 */
let project = null

function request_package_success(pkg_data) {
    project = new ros2_ui_project(JSON.parse(pkg_data))
    package_to_network(project)
}

function display_package_details() {
    $("#pkg_name").text(project.project_info.package_name)
    $("#pkg_description").val(project.project_info.description)
    $("#pkg_version").val(project.project_info.version)
    $("#pkg_license").val(project.project_info.license)
    $("#pkg_maintainer").val(project.project_info.maintainer)
    $("#pkg_maintainer_mail").val(project.project_info.maintainer_mail)
    $("#pkg_pubs").text("" + project.count_pubs())
    $("#pkg_subs").text("" + project.count_subs())
    $("#pkg_topics").text("" + project.count_topics())
}

function buildProject() {
    const queryString = window.location.search;
    const urlParams = new URLSearchParams(queryString);
    if (urlParams.has('project')) {
        const filename = urlParams.get("project")
        $('#build_dialog').get(0).showModal()
        const build_dialog = $('#build_dialog_log')
        build_dialog.empty()
        build_dialog.html(build_dialog.html() + "Sending build request...")
        socket.emit('build', {filename: filename})
        build_dialog.html(build_dialog.html() + "<br>" + "waiting...")
    } else {
        alert("Open Project first!")
    }
}

function close_build_dialog() {
    $('#build_dialog').get(0).close()
}

new MutationObserver(changes => {
    const t = $('#build_dialog_log').parent().get(0)
    t.scrollTop = t.scrollHeight
}).observe($("#build_dialog").get(0), {childList: true, subtree: true})