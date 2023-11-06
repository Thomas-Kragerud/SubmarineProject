const { Client } = require("@notionhq/client");
const { NotionToMarkdown } = require("notion-to-md");
const fs = require('fs');
// or
// import {NotionToMarkdown} from "notion-to-md";

const notion = new Client({
  auth: "secret_ygYQNKlRvqMOtE7j9QKYWvJuhQi6xQgUkZFQheXOU5J",
  config:{
     separateChildPage:true, // default: false
  }
});

const pages = [
  { name: "Readme", token: "ddd34e5b4c4c47caa15961570e6f9695", filePath: "../README.md"},
  //{ name: "Scheduling", token: "8819161ad7a840188d2c8266e1db324a", filePath: "scheduling/scheduling.md" },
  //{name: "Distributed Systems", token: "aec802de8876479983053855e304f549", filePath: "distributed_systems.md"},
  //{name: "Semaphores & Synchronization", token: "675352bdd02b40278698c99e4d0a38a7", filePath: "synchronisation/semaphores_&_synchronization.md"}
];

// NotionToMarkdown instance
const n2m = new NotionToMarkdown({ notionClient: notion });

(async () => {
  for (const page of pages) {
    const mdblocks = await n2m.pageToMarkdown(page.token);
    const mdString = n2m.toMarkdownString(mdblocks);
    fs.writeFileSync(page.filePath, mdString.parent, "utf8");
  }
})();