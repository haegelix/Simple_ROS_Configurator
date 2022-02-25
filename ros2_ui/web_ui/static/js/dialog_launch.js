class DialogLaunch {
    dialog = $("#launch_dialog")
    open() {
        this.dialog.get(0).showModal()
    }
    close() {
        this.dialog.get(0).close()
    }
    launch() {
        this.open()
        request_package_launch()
        $("#launch_log2").val("Launch request was sent...\n")
    }
    stop() {
        request_package_stop()
    }
}

dialog_launch = new DialogLaunch()


document.addEventListener("DOMContentLoaded", function () {
    socket.on('launch', function (message) {
        const msg = message.toString() + "\n"
        const log = $('#launch_log2')
        log.val(log.val() + msg)
        const t = log.get(0)
        t.scrollTop = t.scrollHeight
    })
})

function request_package_launch() {
    const queryString = window.location.search;
    const urlParams = new URLSearchParams(queryString);
    if (urlParams.has('project')) {
        const filename = urlParams.get("project")
        socket.emit('launch', {filename: filename})
    } else {
        alert("Open Project first!")
    }
}

function request_package_stop() {
    socket.emit('stop')
}

