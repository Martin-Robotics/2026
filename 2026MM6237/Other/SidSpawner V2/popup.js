function sleep(ms) {
	return new Promise(resolve => setTimeout(resolve,ms))
}

function getActiveTab() {
	chrome.tabs.query({active: true, currentWindow: true}, (tabs) => {
	if (tabs[0]) {
		const sidsToSpawn = Number(document.getElementById("SidsToSpawn").value)
		const cooldown = Number(document.getElementById("Cooldown").value)
		const endless = document.getElementById("EndlessSids").checked
		spawnSidsLoop(sidsToSpawn, cooldown, tabs[0].id, endless)
	}})
}

async function spawnSidsLoop(sidsToSpawn, cooldown, tabID, endless) {
	while (endless == true) {
		chrome.scripting.executeScript({target: {tabId: tabID}, func: spawnASid})
		await sleep(cooldown)
	}
	while (sidsToSpawn > 0 && endless == false) {
		chrome.scripting.executeScript({target: {tabId: tabID}, func: spawnASid})
		sidsToSpawn -= 1
		await sleep(cooldown)
	}
}

function spawnASid() {
	const imageList = ['sid.png', 'pineapple.png', 'sid2.png']
	const chosenImg = imageList[Math.floor(Math.random()*imageList.length)]
	const newImg = document.createElement('img')
	const imgSize = Math.floor(Math.random()*222)
	let xpos = Math.floor(Math.random()*document.body.scrollWidth)-10

	newImg.src = chrome.runtime.getURL('images/'+chosenImg)
	newImg.zIndex = 100000000000000000000000000000000000000000
	newImg.style.top = `${Math.floor(Math.random()*document.body.scrollHeight)-imgSize}px`
	newImg.style.left = `${xpos}px`
	newImg.style.position = 'absolute'
	newImg.width = `${imgSize}`
	newImg.height = `${imgSize}`
	document.body.appendChild(newImg)
}

function initSidAllImages() {
	chrome.tabs.query({active: true, currentWindow: true}, (tabs)=> {
	if (tabs[0]) {
		chrome.scripting.executeScript({target: {tabId: tabs[0].id}, function: sidAllImages})
	}})
}

function sidAllImages() {
	const allImages = document.getElementsByTagName('img')
	const sidList = ['sid.png', 'sid2.png']
	for (const img of allImages) {
		const chosenSid = sidList[Math.floor(Math.random()*sidList.length)]
		img.src = chrome.runtime.getURL('images/'+chosenSid)
	}
}

document.getElementById('SpawnASidButton').addEventListener('click', getActiveTab)
document.getElementById('SidAllImages').addEventListener('click', initSidAllImages)