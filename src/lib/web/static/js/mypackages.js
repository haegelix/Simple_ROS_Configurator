let socket = io("/package");
let packages_filenames = []
let packages = []

// SETUP START
document.addEventListener("DOMContentLoaded",function (){
    socket.on('connect', function() {
        console.log("WebSocket connected!")
    });

    socket.on('packages_list', function(message) {
        packages_filenames = message.list
    });

    socket.on('package', function (message) {
        packages.push(message.data)
    })
})
// SETUP END


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