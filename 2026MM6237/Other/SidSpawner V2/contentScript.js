spawnSidsFunction

function sleep(ms) {
	return new Promise(resolve => setTimeout(resolve,ms))
}

function createPermSid() {
	const permSid = document.createElement('img')
	let imgSize = Math.floor(Math.random()*222)
	permSid.src = chrome.runtime.getURL('images/sid.png')
	permSid.zIndex = 100000000000000000000000000000000000000000000
	permSid.style.position = 'fixed'
	permSid.width = "100"//`${imgSize}`
	permSid.height = "100"//`${imgSize}`
	permSid.style.top = "0px"
	document.body.appendChild(permSid)
}

var score = 0
let sidSpawnTime = Math.floor(Math.random()*1000)
let spawnSids = true


async function spawnSidsFunction() {
	while (spawnSids == true) {
		const imageList = ['sid.png', 'pineapple.png', 'sid2.png']
		const chosenImg = imageList[Math.floor(Math.random()*imageList.length)]
		const newImg = document.createElement('img')
		let imgSize = Math.floor(Math.random()*222)
		newImg.src = chrome.runtime.getURL('images/'+chosenImg)
		newImg.zIndex = 100000000000000000000000000000000000000000
		newImg.style.top = `${Math.floor(Math.random()*document.body.scrollHeight)-imgSize}px`
		newImg.style.left = `${Math.floor(Math.random()*document.body.scrollWidth)-imgSize}px`
		newImg.style.position = 'absolute'
		newImg.width = `${imgSize}`
		newImg.height = `${imgSize}`
		document.body.appendChild(newImg)
		await sleep(sidSpawnTime)
	}
}

spawnSidsFunction()