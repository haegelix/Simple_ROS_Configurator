let socket = io("/package");
let packages_filenames = []
let packages = []
const package_selection_invalid = document.getElementById("package_selection_invalid")
const package_selection_dropdown = document.getElementById("package_selection")

// SETUP START
document.addEventListener("DOMContentLoaded",function (){
    socket.on('connect', function() {
        console.log("WebSocket connected!")
        request_package_list()
    });

    socket.on('packages_list', function(message) {
        console.log(message.list)
        packages_filenames = message.list
        request_packages_list_success(message.list)
    });

    socket.on('package', function (message) {
        packages.push(message.data)
        console.log(packages)
        request_package_success(message.data)
    })
})
// SETUP END


function request_packages_list_success(list) {
    const curr_select = package_selection_dropdown.value
    while (package_selection_dropdown.hasChildNodes()) {
        package_selection_dropdown.removeChild(package_selection_dropdown.firstChild)
    }

    package_selection_dropdown.appendChild(package_selection_invalid)

    for (let filename of list) {
        let option = document.createElement("option")
        option.value = filename
        option.innerText = filename
        package_selection_dropdown.appendChild(option)
    }

    if (list.includes(curr_select)) {
        package_selection_dropdown.value = curr_select
    } else {
        package_selection_dropdown.value = "invalid"
    }
}

function package_selected(){
    request_package(package_selection_dropdown.value)
}

function request_package_list(){
    socket.emit('get_packages_list')
}

function request_all_packages(){
    for(let filename of packages_filenames){
        request_package(filename)
    }
}

function request_package(filename){
    socket.emit('get_package', {filename: filename})
    console.log("requested " + filename)
}

function request_package_success(pkg_data) {
    console.log("got package")
    console.log(pkg_data)
    document.getElementById("pkg_name").innerText = pkg_data.package_name
    document.getElementById("pkg_description").innerText = pkg_data.package_info.description
    document.getElementById("pkg_version").innerText = pkg_data.package_info.version
    document.getElementById("pkg_license").innerText = pkg_data.package_info.license
    document.getElementById("pkg_maintainer").innerText = pkg_data.package_info.maintainer
    document.getElementById("pkg_maintainer_mail").innerText = pkg_data.package_info.maintainer_mail
    document.getElementById("pkg_pubs").innerText = "" + pkg_data.pubs.length
    document.getElementById("pkg_subs").innerText = "" + pkg_data.subs.length

    //TODO load subs and pubs
    package_to_network(pkg_data)
}
