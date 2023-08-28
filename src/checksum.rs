use merkle_hash::{Algorithm, MerkleTree};
use std::fs::{read_to_string,write};
use crate::printLog;


const ASSETS_PATH: &str = "./assets";
const ASSETS_CHECKSUM_PATH: &str = "./configs/assets_checksum";

pub fn getChecksum() -> String
{
    read_to_string(ASSETS_CHECKSUM_PATH).unwrap()
}

pub fn calcChecksum()
{
    let tree = MerkleTree::builder(ASSETS_PATH)
        .algorithm(Algorithm::Blake3)
        .hash_names(false)
        .build().expect("Failed to calc checksum of assets");
    let checksum = hex::encode(tree.root.item.hash);
    printLog!("Checksum of assets: {}", checksum);
    if checksum != getChecksum()
    {
        write(ASSETS_CHECKSUM_PATH,checksum).unwrap();
        printLog!("Checksum updated!");
    }
    else
    {
        printLog!("Checksum didn't change!");
    }
}

