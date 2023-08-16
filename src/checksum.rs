use merkle_hash::{Algorithm, MerkleTree};
use std::fs::{read_to_string,write};

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
    println!("Checksum of assets: {}", checksum);
    if checksum != getChecksum()
    {
        write(ASSETS_CHECKSUM_PATH,checksum).unwrap();
        println!("Checksum updated!");
    }
    else
    {
        println!("Checksum didn't change!");
    }
}

